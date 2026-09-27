# Occupancy-grid map format

## Baseline format

The first implementation should support a small human-readable ASCII format. A map consists of a header followed by a `data` marker and exactly `height` rows.

```text
resolution 0.10
origin 0.0 0.0
width 20
height 12
data
####################
#..................#
#....####..........#
#..................#
#..............G...#
#..................#
#...S..............#
#..................#
#..................#
#..................#
#..................#
####################
```

## Header fields

| Field | Meaning | Validation |
| --- | --- | --- |
| `resolution` | Cell width and height in metres | Finite and greater than zero |
| `origin` | World coordinate of the lower-left map corner | Two finite values |
| `width` | Number of columns | Positive integer; must match each row |
| `height` | Number of rows | Positive integer; must match row count |
| `data` | Start of map rows | Must occur once after the header |

Header keys are case-sensitive in the baseline format. Blank lines before `data` may be ignored; blank map rows are not valid.

## Cell symbols

- `.` — free cell
- `#` — occupied cell
- `S` — optional start marker; at most one
- `G` — optional goal marker; at most one

`S` and `G` are traversable for map loading but are converted to free cells after their cells are extracted. The current CLI runner and viewer require one of each; lower-level planner APIs can instead receive explicit world or grid poses.

Unknown symbols, missing rows, extra rows, inconsistent row widths, and duplicate markers are errors.

## Coordinate convention

Rows are written from highest world `y` to lowest world `y`, so the first data row is the top row in a visual display. For a cell at column `c` and file row `r`:

```text
cell_center_x = origin_x + (c + 0.5) * resolution
cell_center_y = origin_y + (height - r - 0.5) * resolution
```

World-to-cell conversion must reject points outside the half-open map bounds:

```text
origin_x <= x < origin_x + width * resolution
origin_y <= y < origin_y + height * resolution
```

## Planning and footprint rules

The planner should use a derived inflated occupancy grid. A free cell is planning-safe only if the configured robot footprint can remain inside free space around that cell. The original map must remain available for visualization and final collision checks.

The baseline footprint is a circle with configurable radius. Inflation may conservatively mark every cell whose centre is within `robot_radius` plus one cell diagonal of an obstacle. The exact conservative margin must be documented in code and covered by tests.

## Map authoring guidance

- Keep a solid occupied border around maps used for demos.
- Use at least one cell of clearance beyond the robot footprint for valid corridors.
- Start with maps small enough to inspect by eye.
- Keep `S` and `G` away from obstacle boundaries.
- Store maps with Unix line endings and no trailing spaces.
