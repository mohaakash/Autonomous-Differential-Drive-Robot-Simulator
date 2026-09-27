# Differential-drive kinematics

## Conventions

- Position is measured in metres.
- Heading and angular velocity are measured in radians and radians/second.
- Wheel angular velocity is measured in radians/second.
- Positive wheel velocity drives the robot forward.
- `x` is forward when `theta = 0`; `y` is left of the robot when `theta = 0`.
- The wheelbase `L` is the distance between the left and right wheel contact lines.
- The wheel radius is `r`.

## Forward kinematics

For left and right wheel angular velocities `omega_L` and `omega_R`:

```text
v     = r / 2 * (omega_R + omega_L)
Omega = r / L * (omega_R - omega_L)
```

The body twist is `(v, Omega)`. In the world frame:

```text
dx/dt     = v * cos(theta)
dy/dt     = v * sin(theta)
dtheta/dt = Omega
```

Equal wheel velocities produce translation. Equal and opposite velocities produce rotation around the robot centre.

## Inverse kinematics

For a desired body twist `(v, Omega)`:

```text
omega_R = (v + L / 2 * Omega) / r
omega_L = (v - L / 2 * Omega) / r
```

The implementation calculates both wheel commands before applying limits. If either exceeds `max_wheel_rad_s`, it scales both commands by the same factor:

```text
scale = min(1, max_wheel_rad_s / max(abs(omega_L), abs(omega_R)))
```

Uniform scaling preserves the requested curvature. The returned command must indicate that clipping occurred.

## Pose integration

For a step `dt`, the implementation uses the exact constant-twist update:

```text
if abs(Omega) < epsilon:
    x_new     = x + v * cos(theta) * dt
    y_new     = y + v * sin(theta) * dt
    theta_new = theta + Omega * dt
else:
    radius    = v / Omega
    dtheta    = Omega * dt
    x_new     = x + radius * (sin(theta + dtheta) - sin(theta))
    y_new     = y - radius * (cos(theta + dtheta) - cos(theta))
    theta_new = theta + dtheta
```

Use a small `epsilon` to avoid division by a near-zero angular velocity. This keeps curved-path motion deterministic without the extra error of an Euler-only update.

Normalize `theta_new` to the project-wide interval `[-pi, pi)` after every update.

## Validation rules

Reject or fail fast for:

- Non-finite `r`, `L`, `dt`, or command values
- `r <= 0`, `L <= 0`, or `dt <= 0`
- Negative maximum speed limits
- Robot dimensions that are inconsistent with the selected footprint model

The simulation should not silently replace invalid values with defaults.

## Numerical invariants and tests

- Zero wheel speeds leave the pose unchanged.
- Equal wheel speeds do not change heading except for numerical noise.
- Opposite wheel speeds produce approximately zero translation for a centred robot.
- Forward and inverse kinematics round-trip within a documented tolerance.
- Applying the same command for the same number of fixed steps gives the same result for the same initial state.
- Heading normalization always returns a value in `[-pi, pi)`.

## Saturation layers

Apply limits in this order:

1. Controller limits body linear and angular velocity.
2. Inverse kinematics converts the twist to wheel speeds.
3. Wheel limits uniformly scale the wheel pair if necessary.
4. The executed twist may be recomputed from the clipped wheel command for accurate metrics.

This distinction lets the report show both the requested command and the command that was physically executed by the model.
