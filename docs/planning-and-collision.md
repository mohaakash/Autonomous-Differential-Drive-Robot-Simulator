# Planning and collision behavior

## Planning pipeline

1. Load the original occupancy grid.
2. Validate start and goal world poses.
3. Inflate occupied cells using the robot footprint.
4. Convert start and goal to grid cells.
5. Run A* on the inflated grid.
6. Reconstruct the path from goal to start.
7. Convert cell centres into world-frame waypoints.
8. Validate the resulting path against the collision model before execution.

The planner must never return a path that crosses an occupied or inflated cell.

The current planner derives inflated occupancy by evaluating the configured circular footprint at every cell centre. It then applies the collision checker to every candidate grid transition, so diagonal moves cannot cut through obstacle corners. This keeps the planner and final pose validation on the same collision rules.

## A* baseline

Use an 8-connected neighbourhood. Orthogonal moves cost `1`; diagonal moves cost `sqrt(2)`. The heuristic should be the octile distance:

```text
dx = abs(goal.column - cell.column)
dy = abs(goal.row - cell.row)
h  = (dx + dy) + (sqrt(2) - 2) * min(dx, dy)
```

This heuristic is admissible for the stated move costs. Use a priority queue ordered by lowest `f = g + h`, with deterministic tie-breaking by `f`, then `h`, then row/column order. Deterministic tie-breaking makes saved paths reproducible.

If the start equals the goal cell, return a valid one-cell path and let the controller evaluate the pose tolerances.

## Unreachable and invalid cases

Return a structured failure for:

- Start or goal outside the map
- Start or goal in inflated occupied space
- No route after exploring all reachable cells
- Malformed or empty map

Do not substitute the nearest free cell silently. If nearest-cell recovery is added later, it must be an explicit scenario option and must be reported.

## Path post-processing

The first implementation may use the raw cell-centre path. Optional post-processing can:

- Remove collinear intermediate cells.
- Replace short zig-zags with line segments.
- Smooth corners while preserving a collision-clearance margin.

Every post-processed segment must be sampled or intersected against the inflated map. A visually smooth path that cuts through an obstacle is invalid.

## Collision model

The baseline robot footprint is a circle centred at `(x, y)` with radius `robot_radius`. A pose is collision-free when the circle is inside map bounds and does not intersect any occupied cell geometry.

For a discrete map, a conservative implementation may check all occupied cells within `robot_radius` of the robot centre. A more exact implementation may use circle-to-axis-aligned-rectangle distance:

```text
closest_x = clamp(robot_x, cell_min_x, cell_max_x)
closest_y = clamp(robot_y, cell_min_y, cell_max_y)
distance  = hypot(robot_x - closest_x, robot_y - closest_y)
collision if distance < robot_radius
```

Use one collision predicate for final pose validation, path validation, and LiDAR obstacle geometry where practical.

## Motion-segment checking

Checking only the final pose can miss a collision during a large step. The simulator should sample the candidate motion segment at a spacing no larger than half a map cell or use continuous geometry. If any sample is invalid:

1. Do not commit the unsafe pose.
2. Set the executed command to zero.
3. Record the rejected candidate and collision location if available.
4. End the run or invoke the configured recovery policy.

## Replanning triggers

Replan when any of these is true:

- The active path contains newly occupied or unsafe cells.
- A LiDAR return is closer than the configured safety distance in the path corridor.
- The controller has made no progress for the configured timeout.
- A recovery policy explicitly requests a replan.

Limit repeated replanning with a maximum count or cooldown and report every replan in the run metrics.
