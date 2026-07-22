# Painting Executor Motion Translation

The active executor is implemented in:

`ros2/robross_painter/src/painting_executor.cpp`

It does not send `painting_paths` commands directly to the Aubo robot. Instead,
it translates them through several layers:

```text
painting_paths.json
-> canvas-space pen-tip poses
-> ee_link poses
-> MoveIt Cartesian or joint-space trajectories
-> timed joint trajectories
-> MoveIt ExecuteTrajectory
-> joint_trajectory_controller
-> ros2_control position commands
-> Aubo servoJoint()
```

The executor maintains a small logical state machine tracking:

- Whether this is the first robot motion.
- Whether a valid canvas position has been established.
- Whether the pen is logically up or down.
- The current logical `(x, y)` position on the canvas.

Those initial values are defined at
`ros2/robross_painter/src/painting_executor.cpp:1943-1947`.

## 1. Loading the Path File

The launch file gives the executor:

- The `painting_paths.json` filename.
- Robot description and kinematics configuration.
- Canvas calibration.
- Tool offset.
- Collision objects.
- Motion and safety limits.

The executor loads the complete JSON and validates it before dispatching any
commands at `ros2/robross_painter/src/painting_executor.cpp:696-772`.

It requires:

- A `commands` array.
- Positive finite `canvas.width_mm` and `canvas.height_mm` values.
- A recognized command name.
- Finite coordinates.
- Exactly two coordinates in each point.
- At least two points for `paint_path`.

The recognized command types are:

```text
select_tool
dip_paint
move_to
lower_tool
lift_tool
paint_stroke
paint_path
```

Unknown commands are rejected during loading, despite an unreachable
dispatcher branch that says it will skip them.

### Fields That Affect Motion

The executor uses:

- `canvas.width_mm`
- `canvas.height_mm`
- `commands`
- `path_settings.tool_width_mm`, only for RViz line thickness

It ignores for motion:

- `units`
- `canvas.origin`
- `canvas.margin_mm`
- `color`
- `version`
- `validation`
- Most `path_settings`
- Debug and source metadata

Consequently, coordinates are always interpreted as millimeters from the
canvas's top-left corner, even if the JSON metadata says otherwise.

For example:

```json
{
  "command": "move_to",
  "x_mm": 58.08,
  "y_mm": 10.0
}
```

This means: put the physical pen tip 58.08 mm to the right and 10 mm down from
the calibrated top-left corner.

## 2. Establishing the Canvas Frame

The canvas calibration defines a 3D coordinate frame:

- Canvas X: page right.
- Canvas Y: page down.
- Canvas Z: into the paper.
- Origin: top-left corner on the paper surface.

This convention is implemented in `CanvasFrame` at
`ros2/robross_painter/src/painting_executor.cpp:94-147`.

For a path coordinate `(x_mm, y_mm)` and a normal offset `z_off`, the desired
pen-tip position is:

```text
p_tip = p_origin
      + (x_mm / 1000) * X_canvas
      + (y_mm / 1000) * Y_canvas
      - z_off * Z_canvas
```

The implementation is at
`ros2/robross_painter/src/painting_executor.cpp:130-145`.

Because canvas Z points into the paper:

- `z_off = 0` means the pen tip is on the calibrated paper plane.
- Positive `z_off` moves the tip away from the paper.
- `safe_clearance_m`, normally 0.02 m, means 20 mm above the paper.
- There is no negative penetration or press depth.

The default RViz wall calibration uses:

```yaml
canvas_origin_xyz: [0.55, 0.105, 0.55]
canvas_quat_xyzw: [0.5, -0.5, 0.5, -0.5]
safe_clearance_m: 0.02
```

See `ros2/robross_painter/config/rviz_wall_a4.yaml:7-19`.

A taught canvas file can override the origin and quaternion, allowing the same
path coordinates to be applied to a differently positioned or tilted canvas.

## 3. Determining Tool Orientation

The pen orientation is constant throughout the painting.

The desired tip orientation is:

```text
Q_tip = Q_canvas * Q_spin
```

`Q_spin` is a rotation around the pen axis from `tool_spin_deg`. This is
calculated at `ros2/robross_painter/src/painting_executor.cpp:182-190`.

This means:

- The pen axis remains normal to the canvas.
- `tool_spin_deg` rotates the claw around the pen axis.
- Individual commands cannot change orientation.
- Paths contain only 2D positions, not orientations.

## 4. Converting Pen-Tip Poses to Robot Poses

Path coordinates describe the physical pen tip, but MoveIt plans motion for
`ee_link`.

The calibration provides the fixed transform from `ee_link` to the pen tip:

```yaml
tool_offset_xyz: [0.0, -0.0595, 0.0514]
tool_offset_rpy: [0.0, 0.0, 0.0]
```

The executor converts the desired tip pose into an end-effector pose using:

```text
T_base_ee = T_base_tip * inverse(T_ee_tip)
```

This happens in `makePose()` at
`ros2/robross_painter/src/painting_executor.cpp:786-803`.

If the pen is mounted 59.5 mm sideways and 51.4 mm away from `ee_link`, MoveIt
moves `ee_link` to the offset location needed to put the pen tip on the
requested canvas point.

MoveIt does not directly plan the tip frame because the tip is not an
articulated robot link. It plans `ee_link` while accounting for the calibrated
rigid offset.

## 5. Command State Machine

Commands are executed strictly in array order at
`ros2/robross_painter/src/painting_executor.cpp:357-390`.

| Command | Required state | Result |
| --- | --- | --- |
| `select_tool` | None | No motion |
| `dip_paint` | None | No motion |
| `move_to` | Pen must be up | Travels to a point at safe clearance |
| `lower_tool` | A previous `move_to` must have succeeded | Moves straight toward the paper |
| `lift_tool` | A previous `move_to` must have succeeded | Moves straight away from the paper |
| `paint_stroke` | Pen must be down | Draws one straight segment |
| `paint_path` | Pen must be down | Draws one continuous polyline trajectory |

### `select_tool`

Currently this is a no-op:

```cpp
if (type == "select_tool" || type == "dip_paint") {
    // Pen demo v1: nothing to do, the pen is always mounted.
}
```

See `ros2/robross_painter/src/painting_executor.cpp:366-369`.

The executor does not:

- Operate a tool changer.
- Send digital I/O.
- Change the physical pen.
- Use the command's `color`.

### `dip_paint`

This is also a no-op. There is no movement to a paint container and no dipping
actuator.

These commands remain in the JSON because they describe higher-level painting
intent, but the current pen demo assumes a permanently mounted pen.

## 6. `move_to`: Pen-Up Travel

Implementation: `ros2/robross_painter/src/painting_executor.cpp:806-842`.

The executor first checks:

```text
0 <= x <= canvas width
0 <= y <= canvas height
```

The boundaries are inclusive.

It then creates a target pose at:

```text
(x_mm, y_mm, safe_clearance_m)
```

If the pen is logically down, the command is rejected. It never automatically
lifts before traveling.

### First `move_to`

The first robot motion uses joint-space planning:

```cpp
ok = moveJointSpace(target);
```

This is necessary because the robot can initially be in an arbitrary pose. A
straight Cartesian path from that arbitrary pose to the canvas may be
impossible or unsafe.

### Later `move_to` Commands

Later travel first attempts a straight Cartesian line at the hover height:

```cpp
ok = moveCartesian({target});
```

If that fails, the executor is allowed to use joint-space planning because the
pen is up.

A joint-space fallback is collision checked, but it is not constrained to stay
exactly 20 mm above the canvas between endpoints. The robot may arc around
obstacles or change height.

On success, `move_to` updates the logical canvas position and sets
`have_position = true`.

## 7. `lower_tool`: Approaching the Paper

Implementation: `ros2/robross_painter/src/painting_executor.cpp:844-858`.

`lower_tool` requires an established current position. Normally that comes from
`move_to`.

It requests:

```text
current X
current Y
z_off = 0
```

The result is a straight Cartesian motion along the canvas normal from hover
height to the calibrated contact plane.

No force or contact sensing is involved. "Pen down" means the tip has reached
the mathematically calibrated plane.

There is no:

- Force-control mode.
- Torque threshold.
- Surface probing.
- Pen compression compensation.
- Additional press depth.

After successful execution, `pen_down` becomes true.

## 8. `lift_tool`: Leaving the Paper

`lift_tool` uses the same vertical-motion function but requests:

```text
current X
current Y
z_off = safe_clearance_m
```

This creates a straight Cartesian movement away from the paper.

When lifting from contact, the trajectory is treated as a painting/contact
motion and receives the stricter guarded-joint limits. Only after successful
completion does `pen_down` become false.

## 9. `paint_stroke`: One Straight Segment

Implementation: `ros2/robross_painter/src/painting_executor.cpp:860-895`.

A stroke has:

```json
{
  "command": "paint_stroke",
  "from_mm": [10.0, 20.0],
  "to_mm": [30.0, 40.0]
}
```

Both endpoints must be inside the canvas, and the pen must already be down.

### Start-Point Reconciliation

The executor compares `from_mm` with its logical current position.

If the difference is at most 0.5 mm, it assumes they are the same point and
starts the stroke from the robot's current position.

If the difference exceeds 0.5 mm, it warns and performs a pen-down Cartesian
movement to `from_mm`:

```text
current contact point
-> declared from_mm
-> declared to_mm
```

That correction deliberately drags the pen across the paper. It does not lift
and reposition.

After reaching `from_mm`, the executor separately plans and executes a straight
Cartesian trajectory to `to_mm`.

Each `paint_stroke` is independently:

- Planned.
- Guard checked.
- Retimed.
- Collision checked.
- Executed.
- Endpoint checked.

Adjacent `paint_stroke` commands therefore introduce command-level trajectory
boundaries, even if the pen remains down.

## 10. `paint_path`: Continuous Polyline

Implementation: `ros2/robross_painter/src/painting_executor.cpp:897-953`.

A path has:

```json
{
  "command": "paint_path",
  "points_mm": [
    [10.0, 20.0],
    [30.0, 40.0],
    [50.0, 25.0]
  ]
}
```

All points must be inside the canvas, and the pen must already be down.

The executor constructs one Cartesian waypoint list containing all segments.

Normally, the first point equals the current position established by:

```text
move_to(first point)
lower_tool
paint_path(first point, ...)
```

In that case, the first point is not added as another target. Points two through
N become Cartesian waypoints.

If the first point differs from the current position by more than 0.5 mm, it is
included as an initial pen-down connector.

The entire polyline is submitted to MoveIt as one Cartesian path and retimed as
one trajectory. This avoids explicit stops between separate painting commands.
MoveIt may still slow at sharp corners due to joint limits and time
parameterization; the code does not guarantee a fixed nonzero corner velocity.

This command is also the intended representation for curves: the curve is
sampled into many `points_mm` positions.

## 11. Joint-Space Motion Generation

Joint-space planning is used for:

- The first `move_to`.
- A later pen-up travel whose straight Cartesian path fails.

The process begins with inverse kinematics in `computeIkJointGoal()` at
`ros2/robross_painter/src/painting_executor.cpp:1467-1561`.

### IK Candidate Selection

The executor seeds IK using:

- The robot's current elbow position.
- 25% through the allowed elbow range.
- 50% through the allowed elbow range.
- 75% through the allowed elbow range.

Each IK attempt has a 0.2-second timeout.

For joints that allow equivalent revolutions, it considers `+/-2*pi`
representations and chooses the value nearest the current measured joint
position. This prevents selecting a mathematically equivalent solution that
requires almost a full joint rotation.

Candidates are rejected if:

- IK fails.
- The elbow leaves the configured elbow-up band.
- A guarded joint's goal displacement is too large.

Among valid candidates, the executor selects the IK solution with the smallest
squared joint displacement from the current state.

### OMPL Planning

For the selected joint target, the executor asks MoveIt to produce four
joint-space plans at
`ros2/robross_painter/src/painting_executor.cpp:1724-1776`.

Each plan is checked against:

- Elbow constraints.
- Guarded-joint endpoint displacement.
- Total guarded-joint travel.
- Maximum step between samples.

The valid plan with the least total joint travel is executed.

This is not an unconstrained "move to pose" request. The executor explicitly
computes a bounded IK joint goal first, then asks MoveIt to plan to that joint
configuration.

## 12. Cartesian Motion Generation

Cartesian planning is used for:

- Later straight hover travels.
- Lowering.
- Lifting.
- Start-point correction drags.
- `paint_stroke`.
- `paint_path`.
- Emergency retreat.

The core call is at
`ros2/robross_painter/src/painting_executor.cpp:1778-1832`:

```cpp
group_.computeCartesianPath(
    waypoints,
    eef_step_,
    jump_threshold_,
    traj);
```

With the default profile:

```yaml
eef_step_m: 0.005
cartesian_jump_threshold: 2.0
```

MoveIt samples the requested end-effector path at up to approximately 5 mm
Cartesian increments and solves IK along it.

The executor requires at least 99.9% of the requested path to be feasible. A
lower fraction is treated as:

- An IK failure.
- An obstacle.
- A robot-configuration jump.

The nonzero jump threshold is specifically intended to reject sudden
transitions between different IK configurations.

Painting motions never fall back to joint-space planning. If a Cartesian stroke
is infeasible, execution aborts rather than allowing the arm to change posture
while the pen is touching the paper.

## 13. Joint-Space Safety Guards

Every generated trajectory is inspected at
`ros2/robross_painter/src/painting_executor.cpp:1563-1656`.

The executor verifies:

- The trajectory is nonempty.
- Every joint position is finite.
- Every point contains the expected number of joints.
- The elbow remains in the configured elbow-up range.
- All guarded joints are present.
- Guarded-joint start-to-end displacement stays under its limit.
- Total accumulated guarded-joint travel stays under its limit.
- Maximum adjacent trajectory step stays under its limit.

The default RViz profile guards:

```yaml
guarded_joints: [shoulder_joint, wrist3_joint]
max_guarded_joint_goal_delta_deg: 120.0
max_guarded_joint_travel_deg: 150.0
max_guarded_joint_paint_travel_deg: 90.0
max_guarded_joint_step_deg: 45.0
```

Painting and contact motions use the tighter 90-degree total-travel limit.
Pen-up travel uses the 150-degree limit.

## 14. Timing the Cartesian Trajectory

MoveIt's `computeCartesianPath()` produces geometry but does not apply the
executor's velocity scaling correctly by itself.

The executor therefore uses MoveIt's Time-Optimal Trajectory Generation at
`ros2/robross_painter/src/painting_executor.cpp:1807-1824`:

```cpp
TimeOptimalTrajectoryGeneration(
    totg_path_tolerance_,
    controller_sample_dt_);
```

It then calls:

```cpp
computeTimeStamps(
    trajectory,
    velocity_scaling,
    acceleration_scaling);
```

The default RViz profile uses:

```yaml
velocity_scaling: 0.3
acceleration_scaling: 0.3
totg_path_tolerance: 0.01
controller_sample_dt: 0.005
```

TOTG emits exact on-profile joint positions every `controller_sample_dt`
(matched to the joint trajectory controller period). The executor then strips
the velocity, acceleration, and effort arrays before validating or sending the
trajectory: with position-only points, the ROS 2 Humble spline controller
falls back to LINEAR interpolation between samples — the same model the
post-retiming validator checks. With derivatives present it would execute
quintic splines the validator never sees (remediation plan Section 2.3 /
Phase 1). Joint-space travel trajectories are not modified and keep their
planner-side derivatives.

There is no direct painting speed in mm/s. Travel and painting use the same
scaling values. Actual tip speed depends on:

- Path geometry.
- Robot kinematics.
- Joint velocity limits.
- Joint acceleration handling.
- Time parameterization.
- Controller interpolation.

The JSON therefore does not specify or guarantee a constant Cartesian painting
speed.

## 15. Post-Retiming Cartesian Verification

Retiming can slightly modify the joint trajectory geometry, so the executor
recalculates the physical pen-tip path using forward kinematics at
`ros2/robross_painter/src/painting_executor.cpp:1164-1292`.

It:

1. Reconstructs `ee_link` from each joint sample.
2. Applies `tool_offset` to recover the physical pen-tip pose.
3. Interpolates between trajectory points so no validation step exceeds one
   degree of joint movement.
4. Decomposes the deviation from the closest point on the requested Cartesian
   polyline into a signed canvas-normal component (positive = into the paper)
   and a tangential component, and tracks the extremes of each separately.
5. Measures tip-orientation error.

The default limits are:

```yaml
max_cartesian_deviation_mm: 2.0          # tangential
max_cartesian_normal_deviation_mm: 0.2   # signed, both into and out of paper
max_cartesian_orientation_deviation_deg: 2.0
```

A trajectory exceeding any limit is rejected before execution. The normal
limit is far tighter than the tangential one because canvas-normal excursions
are what rip the paper (inward) or lift the pen (outward).

This test checks that the resulting tip stays close to the requested line
geometry. It does not strictly prove that every reference segment is visited in
temporal order because it measures distance to the nearest segment.

## 16. Post-Retiming Collision Validation

The executor also interpolates the retimed trajectory at no more than one
degree of joint movement and calls:

```text
/check_state_validity
```

for every sample. See
`ros2/robross_painter/src/painting_executor.cpp:1096-1162`.

This catches collisions that might appear after retiming or between the
original Cartesian samples.

The planning scene can include:

- A ground plane.
- A canvas-backing or wall box.
- An attached claw collision box.

These are installed before command dispatch.

## 17. Executing the Trajectory

After all checks pass, the executor invokes:

```cpp
group_.execute(traj);
```

at `ros2/robross_painter/src/painting_executor.cpp:1684`.

That sends a blocking MoveIt `/execute_trajectory` action.

The downstream chain is:

```text
painting_executor
-> MoveIt ExecuteTrajectory
-> joint_trajectory_controller/follow_joint_trajectory
-> ros2_control six-joint position interfaces
```

The painting executor itself never publishes `JointTrajectory` messages and
never directly calls the Aubo SDK.

## 18. Translation to Physical Aubo Commands

For real hardware, the ros2_control hardware plugin is:

`../aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp`

When the position controller starts, the driver:

- Checks that freedrive is disabled.
- Initializes commands from measured joint positions.
- Enables Aubo servo mode.

See `aubo_hardware_interface.cpp:157-208`.

At each controller update, it receives interpolated six-joint position
setpoints. When the robot is running and its safety state is normal or reduced,
it sends:

```cpp
servoJoint(
    joint_positions,
    0.2,
    0.2,
    0.01,
    0.1,
    200);
```

See `aubo_hardware_interface.cpp:346-374`.

The final physical command is therefore not "draw from `(58.08, 10)` to
`(58.08, 207.92)`". It is a stream of six-joint position targets generated from
that line by MoveIt.

With fake hardware, the same MoveIt and ros2_control path is used, but the
generic simulated hardware mirrors commanded positions into joint state rather
than contacting a robot.

## 19. Feedback and Endpoint Verification

After execution returns, the executor waits for a newer `/joint_states`
message.

It then checks:

- MoveIt reported success.
- Every measured joint is within 2 degrees of the planned endpoint.
- The measured robot state satisfies model and elbow bounds.
- The measured pen-tip position is close to the planned endpoint.
- The measured tip orientation is close to the planned orientation.
- The measured endpoint is collision-free.

See `ros2/robross_painter/src/painting_executor.cpp:1359-1434` and
`ros2/robross_painter/src/painting_executor.cpp:1674-1706`.

The default tip endpoint limits are:

```yaml
max_execution_tip_error_mm: 1.0
max_execution_tip_orientation_error_deg: 1.0
```

These compare the measured endpoint with the planned endpoint, not directly
with the original JSON point.

In `dry_run` mode, physical execution and measured endpoint validation are
skipped. The planned endpoint is used as the synthetic state for the next
command.

## 20. Failure and Retreat Behavior

If any command fails:

- Remaining commands are not attempted.
- The executor logs the command index, type, and label.
- It attempts a retreat if the pen might be down.
- The process exits with status 1.

The retreat logic is at
`ros2/robross_painter/src/painting_executor.cpp:955-994`.

The executor takes the currently measured end-effector pose and requests a
straight Cartesian move away from the canvas by `safe_clearance_m`.

It deliberately does not use joint-space fallback while the pen may be touching
the paper. If the straight retreat cannot be planned, the operator must jog the
robot clear manually.

A file that ends while `pen_down == true` is also considered a failed run. The
executor attempts a retreat rather than silently finishing.

## 21. Worked Example

The beginning of `output/painting_paths.json` is effectively:

```text
select_tool black
dip_paint black
move_to (58.08, 10.0)
lower_tool
paint_path [(58.08, 10.0), (58.08, 207.92)]
lift_tool
```

The translation is:

1. `select_tool` produces no robot action.
2. `dip_paint` produces no robot action.
3. `move_to` creates a pen-tip target 58.08 mm right, 10 mm down, and 20 mm
   away from the calibrated canvas.
4. The desired tip target is transformed into an `ee_link` target using the
   calibrated pen offset.
5. Because this is the first motion, bounded IK and joint-space planning are
   used.
6. The selected joint trajectory is executed and checked against measured
   feedback.
7. `lower_tool` creates a straight Cartesian target at the same `(x, y)` with
   zero normal offset.
8. Cartesian IK samples the lowering motion, applies guards, retimes it, checks
   FK deviation and collisions, and executes it.
9. `paint_path` creates one straight Cartesian segment from `(58.08, 10.0)` to
   `(58.08, 207.92)` on the contact plane.
10. The segment is converted into sampled robot joint positions, retimed,
    validated, and sent to the joint trajectory controller.
11. `lift_tool` moves the tip straight away from the paper to the 20 mm
    clearance plane.
12. Only after successful lifting can the next `move_to` perform pen-up travel.

## 22. Most Important Practical Consequences

- `select_tool`, `dip_paint`, and `color` currently have no physical effect.
- The executor assumes all path coordinates are top-left-origin millimeters.
- Contact is positional, not force controlled.
- There is no explicit pen pressure.
- The first travel uses joint-space planning.
- Later travel tries straight Cartesian motion first.
- Pen-down motion is Cartesian only.
- `paint_path` is one trajectory; separate `paint_stroke` commands are separate
  trajectories.
- A start mismatch greater than 0.5 mm causes a pen-down drag to the declared
  start.
- Speed is joint-limit-based, not a specified mm/s painting speed.
- Every Cartesian trajectory is retimed, FK checked, collision checked, and
  endpoint checked.
- The actual Aubo receives interpolated joint-position setpoints through
  `servoJoint()`, not canvas commands.
