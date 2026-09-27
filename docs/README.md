# Differential-drive simulator documentation

This directory contains the current design, implementation contracts, and verification documents for the C++17 2D autonomous differential-drive robot simulator. The repository is implemented through Phase 8; the roadmap and requirements documents identify genuinely deferred extensions.

The project specification is in [autonomous-differential-drive-robot-simulator.md](autonomous-differential-drive-robot-simulator.md). The documents below turn that specification into implementation contracts. They describe the intended baseline; the implementation may refine details, but changes should be recorded in the relevant document.

## Document map

| Document | Purpose |
| --- | --- |
| [requirements-and-acceptance.md](requirements-and-acceptance.md) | Scope, requirements, assumptions, and definition of done |
| [architecture.md](architecture.md) | Module boundaries, data flow, and simulation-loop order |
| [kinematics.md](kinematics.md) | Differential-drive equations, integration, limits, and invariants |
| [coordinate-frames.md](coordinate-frames.md) | World, robot, and LiDAR frames and transform conventions |
| [map-format.md](map-format.md) | ASCII occupancy-map format and coordinate conversion |
| [planning-and-collision.md](planning-and-collision.md) | Occupancy inflation, A*, path validation, and collision rules |
| [control-and-simulation.md](control-and-simulation.md) | Pure-pursuit control and fixed-step simulation behavior |
| [sensor-model.md](sensor-model.md) | Simulated LiDAR ray casting, noise, and safety response |
| [testing-and-validation.md](testing-and-validation.md) | Unit, integration, determinism, and acceptance tests |
| [experiments-and-metrics.md](experiments-and-metrics.md) | Reproducible experiments, output schema, and reporting |
| [interactive-viewer.md](interactive-viewer.md) | Optional raylib replay viewer, controls, and build workflow |
| [implementation-roadmap.md](implementation-roadmap.md) | Ordered implementation phases and exit criteria |

## Reading order

For implementation, read the requirements, architecture, and kinematics documents first. Then read the map/planning, control, and sensor contracts before changing the simulation loop. The testing, experiments, and interactive-viewer documents describe the current workflows and verification gates.

## Status vocabulary

- **Required** means the first standalone simulator must support it.
- **Optional** means it is deliberately deferred until the required baseline is reliable.
- **Proposed** means a concrete default chosen to remove ambiguity; it can be changed if the implementation records the decision and updates affected tests.

## Project-wide conventions

- C++17 is the baseline language standard.
- Angles are in radians internally; distances are in metres; time is in seconds.
- Positive `x` points right/east in the map, positive `y` points up/north, and positive heading rotates counter-clockwise.
- Simulation time is fixed-step and deterministic when the same configuration and random seed are used.
- Invalid input should produce an actionable error rather than silently selecting a fallback.
