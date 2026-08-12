# Painting Executor Motion Translation

**Status:** Stable architecture overview

**Current behavior:** `../ros2/robross_painter/README.md`

**Current parameters:** `../ros2/robross_painter/REFERENCE.md`

**Path contract:** `painting-paths-format.md`

This document explains the coordinate and planning boundaries without copying
profile values, launch commands, or source line numbers. Use the linked
canonical documents for operational details.

## Coordinate Chain

Artwork paths describe the physical pen tip in canvas millimeters:

```text
path point (x_mm, y_mm, z_offset)
    -> canvas frame in base coordinates
    -> desired pen-tip pose
    -> inverse fixed tool transform
    -> desired end-effector pose
```

`CanvasFrame` maps canvas X/Y/Z into the robot base frame. Canvas `+Z` points
into the drawing surface. The taught canvas origin may already include the
reviewed spring preload; executor-local `z_offset == 0` therefore means the
qualified contact plane, not necessarily an unloaded physical paper surface.

The fixed transform from `ee_link` to the pen tip comes from the selected
calibration profile. `makePose()` applies its inverse so MoveIt can plan the
end-effector pose that places the physical tip at the requested canvas point.
Measured tool values belong in reviewed profiles, not explanatory prose.

## Command Translation

`painting_executor` validates the whole input before motion and enforces its
pen-state sequence. Unsupported commands are rejected during preflight.

| Command | Motion meaning |
| --- | --- |
| `move_to` | Move at clearance with the tool raised. |
| `lower_tool` | Cartesian approach along the canvas normal. |
| `paint_stroke` | One straight contact segment. |
| `paint_path` | One continuous contact polyline. |
| `lift_tool` | Cartesian retreat along the canvas normal. |
| `select_tool`, `dip_paint` | Logged no-ops in the current pen executor. |

`doStroke()` and `doPath()` convert canvas points through `makePose()` and pass
Cartesian waypoints to the shared motion pipeline.

## Planning And Validation

Before execution, the painter checks:

- path structure, finite values, bounds, and command sequence;
- scene and collision geometry;
- Cartesian completion and jump policy;
- signed canvas-normal, tangential, and orientation deviation;
- configured motion guards and joint limits.

Cartesian trajectories are resampled at the controller period and stripped to
position-only points so controller interpolation matches the validated model.
The authoritative interval is the selected profile; do not copy a numeric
default into this document.

## Execution Completion

Trajectory action success is not sufficient. Endpoint settling requires a
timeout-bounded sequence of fresh, finite samples that satisfy endpoint error,
low measured velocity, dwell, and minimum-sample requirements. The exact
controls and qualification status live in the painter README, parameter
reference, and hardware status record.

Continuous contact guarding remains a separate safety capability. Endpoint
settling does not authorize unattended contact operation.

## Maintenance Rule

Keep this file conceptual. When implementation changes:

1. Update `ros2/robross_painter/README.md` for behavior.
2. Update `ros2/robross_painter/REFERENCE.md` for parameters.
3. Update `docs/painting-paths-format.md` for command semantics.
4. Update the hardware status and run guide for evidence or procedure changes.
