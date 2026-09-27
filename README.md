# Autonomous Differential-Drive Robot Simulator

Phase 8 interactive viewer and hardened experiment runner for a deterministic 2D autonomous robot simulator. The project specification and implementation contracts are in [`docs/`](docs/README.md).

## Prerequisites

- CMake 3.21 or newer
- A C++17 compiler
- Make or another CMake-supported build tool

Eigen and GoogleTest are optional for the current foundation. CMake uses installed packages when available. To fetch pinned versions automatically, use the `debug-with-fetched-dependencies` preset.

## Build and test

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

The default unit test uses the standard library when GoogleTest is not installed, so the project remains buildable without external packages. To use the dependency-backed setup:

```bash
cmake --preset debug-with-fetched-dependencies
cmake --build --preset debug-with-fetched-dependencies
ctest --preset debug-with-fetched-dependencies
```

## CLI smoke check

```bash
./build/debug/robot_sim --help
./build/debug/robot_sim --version
```

## Run a documented scenario

Maps with `S` and `G` markers can be run directly. The command writes a reproducible artifact bundle containing the resolved configuration, summary metrics, planned path, fixed-step trace, LiDAR samples, and an SVG map overlay:

```bash
./build/debug/robot_sim \
  --run maps/simple_room.map \
  --output results/simple_room_goal \
  --seed 42
```

Open [`results/simple_room_goal/map_overlay.svg`](results/simple_room_goal/map_overlay.svg) in a browser, or inspect the CSV and JSON files from the same directory.

## Hardening checks

Warnings-as-errors, sanitizer, and performance checks are available through the CMake presets:

```bash
cmake --preset ci
cmake --build --preset ci
ctest --preset ci --output-on-failure

cmake --preset asan-ubsan
cmake --build --preset asan-ubsan
ctest --preset asan-ubsan --output-on-failure

./build/ci/robot_sim_benchmark maps/simple_room.map 100
```

The benchmark reports mean, minimum, and maximum wall-clock time for complete deterministic simulation runs. Timing is diagnostic only and is not included in reproducibility comparisons.

## Interactive viewer

The optional raylib viewer replays a deterministic run with the map, planned path, travelled trace, robot pose, and LiDAR rays visible:

```bash
cmake --preset viewer
cmake --build --preset viewer
ctest --preset viewer --output-on-failure
./build/viewer/robot_sim_viewer maps/garden_maze.map
```

Controls: `Space` pauses or resumes, `R` resets, `Left`/`Right` step while paused, `Up`/`Down` change playback speed, and `Escape` exits. The regular `debug` preset remains dependency-free and skips the viewer when raylib is unavailable. See [`docs/interactive-viewer.md`](docs/interactive-viewer.md) for details.

## Simulation recording

The recording below shows the car navigating the garden-maze playground to the goal:

![Differential-drive robot simulation recording](docs/assets/simulation-demo.gif)

## Repository layout

```text
include/robot_sim/  Public project headers
src/                Executable sources
tests/              Unit, integration, and hardening tests
cmake/              Dependency and build helpers
docs/               Design, contracts, and implementation roadmap
```
