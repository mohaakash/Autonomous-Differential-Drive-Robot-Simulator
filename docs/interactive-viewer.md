# Interactive viewer

Phase 8 adds an optional native C++ viewer for inspecting a deterministic map run. The viewer is deliberately a thin presentation layer: planning, control, kinematics, collision checks, and LiDAR remain in `robot_sim_core`, so the headless simulator and its tests do not depend on a graphics library.

## Build

The `viewer` preset fetches the pinned raylib 6.0 dependency, builds the viewer, and runs the normal test suite:

```bash
cmake --preset viewer
cmake --build --preset viewer
ctest --preset viewer --output-on-failure
```

The ordinary `debug` preset can use an installed raylib CMake package when one is present. If raylib is unavailable, it prints a status message and skips `robot_sim_viewer`; the core simulator remains buildable and testable.

## Run

```bash
./build/viewer/robot_sim_viewer maps/garden_maze.map
```

The default `garden_maze.map` contains alternating obstacle barriers with openings on opposite sides, forcing a long serpentine route across the playground. The map must contain exactly one `S` start marker and one `G` goal marker, as required by the scenario runner. The viewer plans and simulates the scenario once, then replays the saved fixed-step trace. Rendering and playback speed do not change navigation behavior.

The scene includes:

- occupied map cells and grid lines;
- the deterministic A* planned path;
- the travelled robot trace;
- start and goal markers;
- the robot footprint and heading;
- sampled LiDAR rays, including no-return rays; and
- frame, simulation time, playback speed, and completion status.

## Controls

| Key | Action |
| --- | --- |
| `Space` | Pause or resume playback |
| `R` | Reset playback to the first frame |
| `Left` | Step one frame backward while paused |
| `Right` | Step one frame forward while paused |
| `Up` | Increase playback speed |
| `Down` | Decrease playback speed |
| `Escape` | Exit the viewer |

This phase is a deterministic trace viewer, not a training environment or a neural-policy visualizer. Learning, population simulation, and video export remain separate future extensions.
