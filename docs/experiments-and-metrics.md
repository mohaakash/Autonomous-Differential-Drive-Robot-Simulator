# Experiments and metrics

## Experiment goals

Experiments should compare configurations and expose failure cases, not only produce a successful screenshot. Each run must be repeatable and must preserve enough information to explain its outcome.

## Scenario matrix

Use a small matrix first:

| Dimension | Example values |
| --- | --- |
| Map | empty room, single obstacle, warehouse-like map |
| Start/goal | straight, turn, narrow passage, unreachable |
| Controller | look-ahead 0.25 m, 0.35 m, 0.50 m |
| LiDAR | disabled noise, low noise, obstacle insertion |
| Safety | stop, replan |
| Seed | 0, 42, 1234 |

Change one meaningful factor at a time when comparing controller behavior.

## Required metrics

Record at both run level and, where useful, step level:

- `status`
- `goal_position_error_m`
- `final_heading_error_rad`
- `max_cross_track_error_m`
- `mean_cross_track_error_m`
- `path_length_m`
- `travelled_distance_m`
- `completion_time_s`
- `simulation_steps`
- `collision_count`
- `replan_count`
- `emergency_stop_count`
- `mean_step_compute_ms`
- `max_step_compute_ms`
- `command_clip_count`
- `success` as a derived boolean

Use the planned path length and travelled distance as separate values. A controller can travel farther than the planned path because of tracking error or replanning.

## Metric definitions

- Position error: Euclidean distance between current and goal position.
- Heading error: absolute wrapped difference between current and goal heading.
- Cross-track error: shortest distance from the current position to the active reference path, measured in world coordinates.
- Path length: sum of distances between consecutive planned waypoints.
- Travelled distance: sum of accepted pose-to-pose displacement; rejected candidate motion is excluded.
- Completion time: simulation time at `GOAL_REACHED`.
- Success rate: completed scenarios divided by total scenarios, excluding invalid-input runs only if the report explicitly says so.

## Output layout

Current output for one scenario:

```text
results/
└── simple_room_goal/
    ├── resolved_config.yaml
    ├── summary.json
    ├── trace.csv
    ├── planned_path.csv
    ├── lidar.csv
    └── map_overlay.svg
```

The current runner writes `summary.json`, `trace.csv`, `planned_path.csv`, `lidar.csv`, the resolved configuration, and `map_overlay.svg`. The SVG is intentionally dependency-free and can be opened directly in a browser. Per-step wall-clock timing fields remain `null` by design; use `robot_sim_benchmark` for environment-dependent runtime measurements without changing deterministic result artifacts.

## Suggested CSV columns

`trace.csv` currently uses:

```text
step,time_s,x_m,y_m,theta_rad,requested_v_m_s,requested_omega_rad_s,
executed_v_m_s,executed_omega_rad_s,left_wheel_rad_s,right_wheel_rad_s,
goal_error_m,heading_error_rad,cross_track_error_m,selected_waypoint_index,
command_limited,collision,safety_blocked,replanned
```

Keep one header row, use invariant decimal formatting, and avoid locale-dependent separators.

## Reporting rules

- Report the number of attempted scenarios as well as successful scenarios.
- Include parameter values beside every comparison.
- State whether measurements use ground-truth pose or an estimated pose.
- Do not average away collisions or failed runs.
- Include limitations and known failure cases.
- Keep raw output; derived plots can be regenerated from it.

## Current command

From the repository root:

```bash
./build/debug/robot_sim --run maps/simple_room.map \
  --output results/simple_room_goal --seed 42
```

The map must contain one `S` start marker and one `G` goal marker. A planning or simulation failure still writes the summary, empty-or-partial CSVs, and visualization so the failure is inspectable.
