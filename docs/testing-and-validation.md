# Testing and validation strategy

Tests should prove mathematics, boundaries between modules, and complete navigation behavior. Keep unit tests deterministic and reserve visual/manual checks for presentation quality.

## Unit-test groups

### Geometry and transforms

- Identity transform leaves a point unchanged.
- Transform followed by inverse transform recovers the original point.
- Rotation by `pi/2` maps the positive `x` axis to positive `y`.
- Angle wrapping handles values just above and below `-pi`/`pi`.
- Grid/world conversion round-trips cell centres.

### Kinematics

- Zero, straight, pure rotation, and curved-motion cases.
- Forward/inverse wheel-speed round trip.
- Wheel-speed saturation preserves left/right ratio.
- Exact integration agrees with a known constant-twist solution.
- Invalid physical parameters are rejected.

### Maps and collision

- Valid maps load with the expected dimensions and markers.
- Malformed headers, symbols, row widths, and duplicate markers fail.
- Boundary poses are classified consistently.
- Circle/rectangle collision checks match hand-calculated cases.
- Inflation blocks corridors narrower than the robot footprint.

### A* planner

- Start equals goal.
- Empty-map route exists.
- Route goes around an obstacle.
- Enclosed goal is unreachable.
- Returned path contains only valid cells.
- Repeated planning has deterministic output.

### Controller and sensors

- Command limits are respected.
- Goal state produces zero command after completion.
- A forward obstacle triggers the safety action.
- LiDAR returns known wall distances.
- Seeded noisy scans reproduce exactly.

### Experiment artifacts

- The documented CLI run reaches the goal on `maps/simple_room.map`.
- The output directory contains resolved configuration, summary JSON, trace CSV, planned-path CSV, LiDAR CSV, and SVG visualization files.
- Repeating the run with the same map, configuration, and seed produces byte-identical artifacts.
- The interactive viewer builds with the `viewer` preset and reaches `goal_reached` on `maps/garden_maze.map` without a collision.

### Hardening and performance

- Malformed maps, sensor parameters, safety parameters, dynamic rectangles, collision spacing, and simulator wiring fail with explicit exceptions.
- The `ci` preset treats compiler warnings as errors.
- The `asan-ubsan` preset runs the complete unit-test suite under AddressSanitizer and UndefinedBehaviorSanitizer.
- `robot_sim_benchmark maps/simple_room.map 100` measures complete simulation runtime without writing artifacts.

The benchmark is intentionally separate from deterministic result generation: wall-clock values are environment-dependent and are never used as navigation metrics.

## Integration tests

At minimum, run these end-to-end cases:

1. Straight route in an empty room.
2. Route around a static obstacle.
3. Unreachable goal with no motion.
4. Newly introduced obstacle with stop policy.
5. Newly introduced obstacle with replan policy.
6. Two identical seeded runs with identical serialized metrics.

Integration tests should assert status, collision count, final errors, and a reasonable upper bound on duration or step count. Avoid asserting every floating-point sample unless the value is part of the public contract.

## Numerical tolerances

Choose tolerances relative to map resolution and simulation step, then keep them in one test configuration. A reasonable starting point is:

- position comparisons: `1e-6 m` for pure math tests;
- angle comparisons: `1e-6 rad` for pure math tests;
- end-to-end goal assertions: the configured goal tolerance, not an arbitrary smaller value;
- collision checks: use a small epsilon consistently at tangency.

The implementation should document any looser tolerance required by the selected integrator.

## Sanitizers and quality gates

During development, run:

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
```

Also run a sanitizer build with AddressSanitizer and UndefinedBehaviorSanitizer where supported. The CI gate includes compiler warnings, unit tests, scenario/artifact smoke coverage, benchmark smoke coverage, sanitizer tests, and documentation checks. A formatter is not pinned yet, so formatting remains a manual review item rather than a silently version-dependent CI gate.

## Reproducibility checklist

Every failing run should preserve:

- map path or map content identifier;
- resolved robot and scenario configuration;
- random seed;
- executable version or commit identifier;
- status and failure reason;
- trace up to the failure;
- the rejected candidate pose for collision failures.

## Manual visual checks

For the interactive demo, run `./build/viewer/robot_sim_viewer` and verify that the display shows the garden maze, car body, start/goal, planned route, travelled trail, LiDAR rays, greenery, and current status. The viewer replays a measured fixed-step result; visual appearance must not be treated as a replacement for the saved metrics.
