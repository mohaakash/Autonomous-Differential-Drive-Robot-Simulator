# Implementation roadmap

This roadmap orders work so every phase leaves a runnable, testable artifact. Do not begin optional ROS2 or localization work until the standalone baseline meets its acceptance criteria.

## Phase 0 — Repository and build skeleton

Create the CMake project, targets, dependency policy, warning flags, test runner, and a command-line executable that can print help.

Exit criteria: clean checkout configures, builds, and runs one smoke test.

## Phase 1 — Geometry and kinematics

Implement `Pose2D`, `Twist2D`, transforms, angle wrapping, forward/inverse wheel kinematics, saturation, and pose integration.

Exit criteria: straight, pure-rotation, curved-motion, round-trip, and invalid-input tests pass.

## Phase 2 — Maps and collision

Implement the ASCII map loader, grid/world conversion, robot footprint, boundary checks, obstacle checks, and motion-segment validation.

Exit criteria: valid maps load, malformed maps fail clearly, and collision tests cover boundaries and footprint clearance.

## Phase 3 — A* planning

Implement obstacle inflation, deterministic 8-connected A*, path reconstruction, path validation, and world waypoint conversion.

Exit criteria: empty, obstacle, and unreachable-map tests pass and no returned segment crosses inflated occupancy.

## Phase 4 — Fixed-step controller

Implement pure pursuit, goal tolerances, velocity limits, fixed-step orchestration, status transitions, and trace recording.

Exit criteria: the robot reaches multiple static-map goals without collision and saved traces include required metrics.

## Phase 5 — LiDAR and safety

Implement transforms for the sensor mount, ray casting, no-return handling, seeded noise, forward safety checks, and stop/replan behavior.

Exit criteria: known-wall sensor tests pass and obstacle insertion produces the expected safety status without a collision.

Status: complete. The baseline includes deterministic LiDAR ray casting, explicit no-return flags, seeded Gaussian noise, a configurable forward-sector monitor, dynamic-obstacle events, stop behavior, and a replan callback contract. Visualization, experiment execution, and persisted result files are covered by the completed later phases.

## Phase 6 — Visualization and experiment runner

Add a lightweight visualization or saved image output, scenario batch execution, CSV/JSON serialization, and a reproducible results directory.

Exit criteria: a new user can run a documented scenario and inspect both a visual artifact and machine-readable metrics.

Status: complete. `robot_sim --run` executes a marker-based map scenario and writes resolved YAML configuration, JSON summary metrics, trace/path/LiDAR CSV files, and a dependency-free SVG map overlay. Identical map, configuration, and seed inputs produce byte-identical artifacts. Wall-clock benchmark timing is kept separate from deterministic result artifacts.

## Phase 7 — Hardening

Add sanitizer runs, malformed-input coverage, performance measurements, documentation examples, and CI checks. Review all reported metrics against raw traces.

Exit criteria: clean build, tests, sanitizer run, and documentation workflow all pass from a clean checkout.

Status: complete. The project now has malformed-input tests, warnings-as-errors and ASan/UBSan presets, a repeatable benchmark executable, documentation workflow checks, and a GitHub Actions CI workflow. Reported experiment metrics are derived directly from the saved simulation result and trace; wall-clock benchmark values remain diagnostic.

## Phase 8 — Interactive C++ visualization

Add an optional native viewer that replays the deterministic simulator while showing the map, planned path, robot trace, pose, and LiDAR output. Keep visualization dependencies outside the core library and preserve a dependency-free headless build.

Exit criteria: the viewer builds through a documented preset, replays a marker-based map scenario, provides basic playback controls, and the default headless build remains usable without raylib.

Status: complete. `robot_sim_viewer` uses raylib 6.0 when available or fetched by the `viewer` preset. It replays the fixed-step trace and renders the garden-maze playground, car body, greenery, planned A* route, travelled trail, LiDAR rays, and live playback metrics. The default `debug` preset continues to build and test without raylib.

## Optional extensions

Only after the baseline is stable:

1. Noisy odometry and an EKF localization comparison.
2. Rectangle footprint and more exact swept collision checks.
3. Controller comparison with PID.
4. Dynamic-obstacle scenarios.
5. ROS2 nodes, messages, launch files, and RViz2 visualization.

Each extension should have its own acceptance tests and should not weaken the deterministic baseline.
