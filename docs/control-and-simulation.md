# Path following and simulation behavior

## Baseline controller

The first controller is pure pursuit. It is easy to inspect, requires few parameters, and maps naturally to a differential-drive body twist.

The controller receives the current pose, a world-frame waypoint path, goal pose, and limits. It returns a body-frame command `(v, Omega)` plus status and error values. The implemented controller is `PurePursuitController`; there is no PID implementation in the current baseline.

## Look-ahead selection

Select the first path point at least `lookahead_distance` ahead of the robot along the path. If no such point exists, use the final waypoint. The selected point should be transformed into `base_link`:

```text
x_r =  cos(theta) * (x_p - x) + sin(theta) * (y_p - y)
y_r = -sin(theta) * (x_p - x) + cos(theta) * (y_p - y)
```

With look-ahead distance `Ld = max(hypot(x_r, y_r), minimum_lookahead)`:

```text
curvature = 2 * y_r / (Ld * Ld)
Omega     = v * curvature
```

The controller should reduce `v` for large curvature and when approaching the goal. All commands are clamped to configured body limits before inverse kinematics.

## Heading and goal behavior

- If the target point is substantially behind the robot, reduce translation and rotate toward the path rather than driving backwards in the baseline mode.
- Within position tolerance, stop translation and rotate in place toward the goal heading.
- Report `GOAL_REACHED` only when both position and heading errors are within tolerance.
- After `GOAL_REACHED`, issue zero commands for all subsequent steps.

The controller must expose at least distance-to-goal, heading error, cross-track error, selected waypoint index, and command saturation state for metrics.

## Implemented Phase 4 baseline

The current implementation provides `PurePursuitController` and `FixedStepSimulator`. The simulator converts the controller twist to wheel commands, recomputes the executed twist after wheel saturation, integrates one fixed step, validates the candidate motion with the collision checker, and records a `TraceSample` after each accepted step. A collision rejects the candidate pose and ends the run.

## Implemented Phase 5 safety extension

`LidarSensor` casts deterministic, evenly spaced rays against occupied map cells and optional axis-aligned dynamic obstacles. The sensor supports a translated and rotated mount, explicit no-return flags, range clamping, and seeded Gaussian range noise. `ForwardSafetyMonitor` evaluates only returns inside a configurable forward sector, so a distant or side-only no-return does not stop the robot.

`FixedStepSimulator` accepts optional dynamic-obstacle activation events and a `SafetyPolicy`. With `Stop`, a blocked forward sector records a zero-command trace sample and finishes with `SAFETY_STOP`. With `Replan`, the simulator invokes a caller-provided callback with the current pose, goal, and active obstacles; an empty or missing result finishes with `REPLAN_FAILURE`, while a valid returned path becomes the active path. Replans and safety stops are counted separately from collisions.

## Fixed-step loop

The simulation uses a configured `dt` independent of rendering and file I/O. A typical step is:

```text
scan          = lidar.measure(true_state, map, rng)
safety_status = safety.evaluate(scan, active_path)
twist         = safety_status.safe ? controller.update(state, path) : zero_twist
wheels        = inverse_kinematics(twist)
candidate     = integrate(state, wheels, dt)
commit        = collision.is_valid_motion(state.pose, candidate.pose)
state         = commit ? candidate : stopped_state(state)
metrics.record(state, scan, twist, wheels, safety_status)
```

The true pose is used by the baseline controller. Noisy odometry/localization is an optional extension and must not be accidentally mixed into the ground-truth path. The viewer renders a replay of the resulting trace; rendering time does not advance the fixed-step simulation.

## Safety response

When the LiDAR or collision module reports an unsafe route:

- `stop` policy: command zero velocity and end with status `SAFETY_STOP` after recording the event;
- `replan` policy: command zero for the current step, update the planning map, and attempt a bounded replan;
- any failed replan ends with `REPLAN_FAILURE`.

Safety takes precedence over goal completion and normal controller output.

## Termination conditions

The run ends with one of:

- `GOAL_REACHED`
- `UNREACHABLE`
- `TIMEOUT`
- `COLLISION`
- `SAFETY_STOP`
- `REPLAN_FAILURE`
- `CONTROL_FAILURE`
- `INVALID_INPUT`

The run status must be serialized even when no motion occurs.
