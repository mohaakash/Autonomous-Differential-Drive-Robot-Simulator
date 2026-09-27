# C++ Autonomous Differential-Drive Robot Simulator — Project Brief

## Project status

Implemented software-only robotics project through Phase 8. It has no physical robot, motors, or external service dependency in the baseline.

## Supporting documentation

This document records the original project brief and design rationale. The current implementation contracts are organized in the [documentation index](README.md), including requirements, architecture, kinematics, coordinate frames, map format, planning, control, LiDAR, testing, experiments, the interactive viewer, and the implementation roadmap. Where this brief differs from those documents, the current contracts and source code are authoritative.

## Project objective

The project provides a 2D autonomous mobile-robot simulator in modern C++. The simulated robot navigates an occupancy-grid map, plans a route to a goal, follows it with pure pursuit, detects obstacles with simulated LiDAR, and reports navigation accuracy.

The project is designed to demonstrate:

- C++17 software engineering
- Differential-drive robot kinematics
- Coordinate frames and transformations
- Linear algebra and numerical reasoning
- A* path planning
- Pure-pursuit control
- Sensor simulation and noisy measurements
- Collision detection and obstacle avoidance
- Automated testing, reproducible experiments, sanitizers, and CI

The current baseline is a self-contained 2D simulator. ROS2, Gazebo, and more advanced localization remain optional extensions after the standalone core.

## Final demonstration

The final demo should show the robot:

1. Loading a documented ASCII occupancy-grid map.
2. Receiving a start pose and target pose.
3. Checking whether the target is reachable.
4. Generating a collision-free path with A*.
5. Converting the path into velocity commands.
6. Moving according to differential-drive kinematics.
7. Using simulated LiDAR readings to detect obstacles.
8. Replanning or stopping when the route becomes unsafe.
9. Displaying the travelled path, target error, heading error, and collision status.

## Scope and design decisions

### In scope

- A 2D robot with position `(x, y)` and heading `theta`
- Two independently driven wheels and one configurable wheelbase
- Static rectangular obstacles
- Occupancy-grid maps
- A* global path planning
- Pure-pursuit local path following
- Simulated LiDAR using ray casting
- Deterministic simulation runs for testing
- Optional ROS2 integration after the standalone version works

### Baseline out of scope

- Physical hardware
- 3D rigid-body dynamics
- Full SLAM
- Real-time operating-system guarantees
- Complex dynamic environments
- Deep-learning-based perception

Keeping the baseline 2D and simulation-only makes the project finishable while still demonstrating important robotics fundamentals.

## Mathematical foundation

### Robot state

Represent the robot pose as:

\[
q = [x, y, \theta]^T
\]

where `x` and `y` are the position in the world frame and `theta` is the heading angle.

### Differential-drive kinematics

Let:

- `r` be the wheel radius
- `L` be the distance between the left and right wheels
- `omega_L` be the left-wheel angular velocity
- `omega_R` be the right-wheel angular velocity

The robot's linear and angular velocity are:

\[
v = \frac{r}{2}(\omega_R + \omega_L)
\]

\[
\Omega = \frac{r}{L}(\omega_R - \omega_L)
\]

The continuous-time motion model is:

\[
\dot{x} = v\cos(\theta)
\]

\[
\dot{y} = v\sin(\theta)
\]

\[
\dot{\theta} = \Omega
\]

For a simulation time step `Delta t`, the implementation uses an exact constant-twist arc update, with a straight-line branch for near-zero angular velocity.

### Inverse wheel kinematics

Given desired robot velocities `v` and `Omega`, calculate the wheel velocities:

\[
\omega_R = \frac{v + \frac{L}{2}\Omega}{r}
\]

\[
\omega_L = \frac{v - \frac{L}{2}\Omega}{r}
\]

The implementation must enforce maximum wheel speed limits and report when a command has been clipped.

### Coordinate frames

Use at least these frames:

- `world`: fixed map coordinate system
- `base_link`: robot center and heading
- `lidar`: simulated sensor frame

Implement 2D rigid transformations so a LiDAR point can be converted from the sensor frame into the world frame. This provides practical experience with rotation matrices, translations, and frame composition.

### Path tracking

The implemented pure-pursuit controller measures cross-track error and heading error, then generates bounded velocity commands. PID comparison remains an optional future extension.

### Optional localization extension

After the basic simulator is complete, add Gaussian noise to odometry and LiDAR measurements. Implement an Extended Kalman Filter to estimate the robot pose and compare:

- Ground-truth pose
- Odometry-only pose
- Filtered pose

This extension is optional and should not delay completion of the core project.

## Recommended technology stack

### Core language and build

- C++17
- CMake
- CMake presets or a documented build script
- Standard Library containers, algorithms, smart pointers, and file I/O

### Mathematics

- Optional Eigen and GoogleTest dependencies through CMake presets
- Custom small geometry utilities for vectors, poses, transforms, rectangles, and collision checks

### Simulation and visualization

The current visualization stack is:

- raylib 6.0 for the optional interactive C++ viewer;
- dependency-free SVG output from the experiment runner; and
- the standard library for the deterministic headless core.

### Testing and quality

- GoogleTest for unit and integration tests
- Git and GitHub
- Compiler warnings such as `-Wall -Wextra -Wpedantic`
- AddressSanitizer and UndefinedBehaviorSanitizer during development

### Optional robotics ecosystem

- ROS2 for nodes, topics, messages, and launch files
- RViz2 for visualization
- Gazebo or another ROS2-compatible simulator for a later comparison

## High-level architecture

```text
Map Loader
    |
    v
Occupancy Grid ---> A* Global Planner ---> Reference Path
                                               |
                                               v
Simulated Sensors ---> Local Controller ---> Wheel Commands
       ^                                      |
       |                                      v
       +----------- Robot Kinematics <--- Pose Integration
                              |
                              v
                    Collision and Metrics
```

Current component boundaries:

1. **Map module** — loads a text, CSV, or image-based occupancy grid.
2. **Robot model** — stores dimensions, limits, pose, and wheel state.
3. **Kinematics module** — performs forward motion integration and inverse wheel calculations.
4. **Planner module** — implements A* and reconstructs a path.
5. **Controller module** — follows the path and limits velocity commands.
6. **Sensor module** — simulates LiDAR rays and measurement noise.
7. **Collision module** — checks the robot footprint against obstacles.
8. **Visualization module** — draws the map, robot, path, rays, and metrics.
9. **Experiment module** — runs repeatable scenarios and saves results.

## Current repository structure

```text
2d-differential-robot/
├── CMakeLists.txt
├── CMakePresets.json
├── README.md
├── include/
│   └── robot_sim/
│       ├── collision/
│       ├── control/
│       ├── experiments/
│       ├── geometry/
│       ├── kinematics/
│       ├── map/
│       ├── planning/
│       ├── safety/
│       ├── sensors/
│       └── simulation/
├── src/
│   ├── collision/
│   ├── control/
│   ├── experiments/
│   ├── kinematics/
│   ├── map/
│   ├── planning/
│   ├── safety/
│   ├── sensors/
│   ├── simulation/
│   ├── benchmark_main.cpp
│   ├── main.cpp
│   └── viewer_main.cpp
├── tests/
│   ├── test_kinematics.cpp
│   ├── test_planning.cpp
│   ├── test_simulation.cpp
│   └── test_hardening.cpp
├── maps/
│   ├── garden_maze.map
│   ├── simple_room.map
│   └── ...
├── docs/
│   ├── implementation-roadmap.md
│   ├── interactive-viewer.md
│   └── ...
└── .github/workflows/ci.yml
```

## Implementation status

The standalone implementation is complete through Phase 8. The current [implementation roadmap](implementation-roadmap.md) is the authoritative phase history and records the completed build skeleton, geometry and kinematics, map and collision model, A* planning, pure-pursuit simulation, LiDAR and safety behavior, experiment artifacts, hardening/CI, and the optional raylib viewer.

The next work is intentionally optional: localization experiments, controller comparisons, richer footprints, dynamic-obstacle scenarios, ROS2 integration, and video export. These should be added only with their own tests and documented acceptance criteria.

## Testing strategy

### Kinematics tests

- Zero wheel velocity keeps the robot stationary.
- Equal wheel velocities create straight-line motion.
- Opposite wheel velocities create pure rotation.
- Inverse and forward kinematics approximately round-trip.
- Wheel-speed limits are enforced.
- Heading normalization handles wraparound correctly.

### Planning tests

- A path exists in an empty map.
- A path goes around an obstacle.
- An unreachable goal is rejected.
- The returned path does not cross occupied cells.
- Inflated obstacles account for the robot footprint.

### Controller tests

- The robot reduces distance to a reachable goal.
- The robot respects maximum linear and angular velocity.
- The controller stops inside the goal tolerance.
- The controller does not command motion after a collision or emergency stop.

### Reproducibility

- Store the resolved simulation parameters with each generated result directory.
- Use fixed random seeds for test scenarios.
- Save every experiment's configuration with its output metrics.

## Evaluation metrics

Report these metrics for several maps and start/goal pairs:

- Goal position error
- Final heading error
- Cross-track error
- Path length
- Completion time
- Number of collisions
- Number of replans
- Benchmark runtime for complete deterministic runs
- Percentage of successfully completed scenarios

Compare controller configurations rather than reporting only a single successful run.

## Portfolio deliverables

- Public GitHub repository
- Build and run instructions
- Unit and integration tests
- Architecture diagram
- Kinematics and coordinate-frame explanation
- Map and configuration examples
- Experiment CSV files or plots
- Short screen recording of navigation
- Section documenting limitations and failed experiments
- Optional ROS2 package and RViz2 screenshot

## Suggested README demo command

The completed repository should aim for a simple workflow such as:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
./build/debug/robot_sim --run maps/simple_room.map \
  --output results/simple_room_goal --seed 42
```

The exact commands may change with the implementation, but a new user should be able to build and run the simulator with only documented open-source dependencies.

## Practical constraints

- Keep the first map small enough to debug visually.
- Begin with perfect odometry before adding noise.
- Begin with static obstacles before adding replanning.
- Prefer simple, tested algorithms over a large unfinished framework.
- Measure behavior instead of claiming that the robot is autonomous because it moves in a demo.
- Do not include metrics in the CV until the experiments produce them.

## CV bullets after completion

Only use these after the implementation and measurements are complete:

> Developed a C++17 differential-drive robot simulator implementing forward and inverse wheel kinematics, exact pose integration, collision checking, and bounded pure-pursuit control with CMake.

> Implemented deterministic A* occupancy-grid path planning and pure-pursuit trajectory tracking, evaluating navigation with path length, cross-track error, completion time, and collision rate.

> Added a simulated LiDAR sensor with coordinate-frame transformations and obstacle-triggered replanning, supported by GoogleTest unit and integration tests.
