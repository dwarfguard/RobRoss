# Aubo Painting Tracking Remediation Plan

**Status:** Proposed implementation plan  
**Created:** 2026-07-22  
**Scope:** Aubo i5 pen-on-paper trajectory execution  
**Related prototype:** `docs/Rob_Ross_Prototype_v1.md`

## 1. Purpose

This plan addresses direction-dependent pen pressure and intermittent endpoint
failures observed during real Aubo i5 painting tests. The immediate goal is to
make the physical pen tip follow the taught canvas plane closely enough that a
fixed spring preload produces continuous lines without tearing the paper.

The work is deliberately staged. Cartesian interpolation, Aubo ServoJ timing,
endpoint settling, and continuous safety monitoring must be changed and
measured independently so that one change does not hide the effect of another.

## 2. Recorded Evidence

The analysis used these ROS 2 bags:

```text
/home/robross/robross_aubo_ws/rosbag2_2026_07_22-20_32_44
/home/robross/robross_aubo_ws/rosbag2_2026_07_22-20_35_14
```

The recordings contain controller reference and feedback, joint states, TF,
MoveIt trajectories, executor logs, and canvas markers. The first bag contains
a complete direction test. The second contains repeated direction tests, the
reversal test, the compact tracking curve, the sine test, and additional curve
tests.

Canvas Z points into the paper. The active canvas calibration uses a `1.0 mm`
plane bias, so an actual modeled tip position of `-1.0 mm` relative to the
biased target corresponds approximately to zero remaining spring compression.

### 2.1 Direction-dependent tracking

| Test region and direction | Mean actual-minus-reference normal error | Estimated mean spring compression |
| --- | ---: | ---: |
| Low-Y `+Y` line, arm retracting | `+0.63 mm` | `1.37 mm` |
| Low-Y `-Y` line, arm extending | `-0.62 mm` | `0.08 mm` |
| High-Y compact curve, `+Y` portions | `+0.18 mm` | `1.16 mm` |
| High-Y compact curve, `-Y` portions | `-0.19 mm` | `0.81 mm` |
| Lower-Y sine, `+Y` portions | `+0.62 mm` | `1.58 mm` |
| Lower-Y sine, `-Y` portions | `-0.66 mm` | `0.34 mm` |
| Horizontal `+X` and `-X` controls | approximately `+0.08/-0.05 mm` | no major directional normal error |

Observed excursions explain both failure modes:

- Low-Y `-Y` motion reached approximately `0.41 mm` beyond contact loss.
- Sine `-Y` motion reached approximately `0.11-0.32 mm` beyond contact loss.
- Sine `+Y` motion reached approximately `2.13 mm` spring compression.

The same-segment and immediate-reversal results rule out paper location, TCP
translation, and a static canvas-plane error as the primary explanation. The
physical video also shows the arm moving lower in `+Y` and higher in `-Y`, in
the same direction predicted by forward kinematics from measured joints.

### 2.2 Joint following error

During the 30 mm Y-direction strokes, maximum joint following errors were
approximately:

| Joint | Maximum error |
| --- | ---: |
| `upperArm_joint` | `0.87 deg` |
| `foreArm_joint` | `1.65 deg` |
| `wrist1_joint` | `0.92 deg` |

The measured joints follow the controller reference with a median common delay
of approximately `100-110 ms` on Y strokes and the sine path. Approximately 95
percent of the joint error resembles a delayed version of the reference, but
the remaining coordination error is enough to create millimeter-scale motion
normal to the paper.

The same error has a larger normal effect when the arm is extended:

| Workspace region | Typical signed normal tracking error |
| --- | ---: |
| Low Y, arm extended | approximately `+/-0.62 mm` |
| Middle Y, reversal test | approximately `+/-0.45-0.50 mm` |
| High Y, arm closer to base | approximately `+/-0.18 mm` |

This is a configuration-dependent Jacobian amplification of tracking error,
not justification for a Y-dependent canvas compensation.

### 2.3 Controller interpolation mismatch

The painting executor currently validates linear interpolation between adjacent
trajectory positions in `painting_executor.cpp`. The active
`joint_trajectory_controller` uses `interpolation_method: splines`. ROS 2
Humble uses:

- Linear interpolation for positions only.
- Cubic interpolation for positions and velocities.
- Quintic interpolation for positions, velocities, and accelerations.

TOTG supplies positions, velocities, and accelerations, so the controller
executes quintic splines. Recorded controller references departed from the
canvas plane by as much as approximately `0.5-0.8 mm`, despite executor reports
of only `0.01-0.45 mm`. The executor therefore does not currently validate the
trajectory interpolation that the controller actually executes.

Relevant code and configuration:

```text
ros2/robross_painter/src/painting_executor.cpp:1233-1273
../aubo_ros2_driver/aubo_ros2_driver/config/aubo_controllers.yaml
/opt/ros/humble/include/joint_trajectory_controller/joint_trajectory_controller/trajectory.hpp
```

### 2.4 Servo loop timing

The controller manager is configured for `200 Hz`, but the bags recorded
`/joint_states` at approximately `153-156 Hz`. A live idle measurement was
approximately `161 Hz`, with update intervals between roughly `3 ms` and
`13 ms`. Controller state was published at only `56-60 Hz` despite a configured
`100 Hz` state publication rate.

The hardware interface targets a 5 ms controller period but calls:

```cpp
servoJoint(joint_positions, 0.2, 0.2, 0.01, 0.1, 200);
```

The Aubo SDK documentation states that `t` should match the interval between
successive `servoJoint` calls and that the currently effective parameters are
`q` and `t`. The driver also retries queue-full responses in a blocking loop,
which can delay the entire ros2_control update loop.

Relevant code and documentation:

```text
../aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp:223-247
../aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp:346-374
../aubo_ros2_driver/aubo_ros2_driver/src/aubo_ros2_control_node.cpp:20-56
../aubo_ros2_driver/aubo_ros2_driver/config/aubo_controllers.yaml
../aubo_ros2_driver/aubo_ros2_driver/config/aubo_i5_update_rate.yaml
../../build/aubo_ros2_driver/_deps/aubo_sdk-src/include/aubo/robot/motion_control.h:615-654
```

### 2.5 Endpoint failures

The longer bag recorded endpoint failures between `1.046 mm` and `1.206 mm`
after `lower_tool`, `lift_tool`, and retreat operations. The executor currently
checks the first fresh joint-state sample after MoveIt reports completion. The
controller has no effective per-joint trajectory or goal tolerances, so action
success does not imply that the arm has physically settled.

## 3. Working Diagnosis

The causes are ranked as follows:

1. Primary: delayed and imperfectly coordinated physical joint tracking through the ServoJ streaming path.
2. Secondary: controller spline interpolation differs from the executor's linear validation model.
3. Amplifier: the extended-arm Jacobian maps small coordination errors into larger canvas-normal displacement.
4. Separate issue: endpoint validation samples the arm before it has consistently settled.
5. Residual checks: payload, center of gravity, structural flex, holder flex, and paper backing remain relevant after encoder-visible errors are corrected.

The evidence does not support treating TCP translation, global preload, or a
direction-dependent canvas Z correction as the primary fix.

## 4. Safety Constraints

The following constraints apply throughout implementation and testing:

- Do not increase the global plane bias by `0.3 mm` while directional errors remain.
- Do not add Y-dependent or direction-dependent Z compensation.
- Do not raise `max_execution_tip_error_mm` to suppress endpoint failures.
- Do not guess manufacturer acceleration limits.
- Do not tune ServoJ lookahead or gain before matching and measuring `t` and the call interval.
- Preserve collision checking, backing-plane protection, elbow policy, and guarded-joint limits.
- Run each behavioral change above the paper before permitting pen contact.
- Change one timing or interpolation variable per recorded comparison.
- Record the RobRoss, Aubo driver, and Aubo description revisions for every approved hardware build.

## 5. Phase 0: Reproducible Bag Analysis

### Goal

Turn the one-off bag investigation into a repeatable analysis tool so every
implementation change can be compared against the same metrics.

### Implementation

Add a read-only analysis script under:

```text
ros2/robross_painter/scripts/analyze_tracking_bag.py
```

The script should:

- Read rosbag2 SQLite recordings without replaying them onto the live ROS graph.
- Read the runtime `robot_description` stored in the bag.
- Read the canvas origin, quaternion, tool offset, and plane bias from recorded parameters or explicitly supplied files.
- Segment executions using `painting_executor` command labels and timestamps from `/rosout`.
- Use `/joint_trajectory_controller/controller_state` for reference and actual joint states.
- Compute reference and actual TCP positions with the same calibrated kinematic chain used by MoveIt.
- Project reference, actual, and actual-minus-reference positions onto canvas X, Y, and Z.
- Report per-joint error, Cartesian error, direction, canvas position, speed, and estimated spring compression.
- Report controller-state and joint-state publication rates and timing jitter.
- Export machine-readable CSV plus a concise Markdown or terminal summary.

### Tests

- Verify canvas-axis and TCP math against existing calibration tests.
- Verify sign convention: positive canvas Z is into the paper.
- Verify synthetic `+Y`, `-Y`, `+X`, and `-X` classifications.
- Verify segmentation of all three arm-tracking fixture files.
- Verify that analysis never publishes ROS topics or commands hardware.

### Acceptance Gate

For the July 22 bags, the tool must reproduce the reported mean normal errors
within `0.05 mm` and joint maxima within `0.05 deg`.

## 6. Phase 1: Make Executed Interpolation Match Validation

### Goal

Ensure the joint trajectory controller executes the same interpolation that the
painting executor validates.

### Preferred Implementation

For Cartesian trajectories only:

1. Retain TOTG to assign safe timestamps.
2. Resample trajectory positions at a configurable controller-oriented period, initially `0.005 s`.
3. Clear velocity and acceleration arrays before sending the trajectory to the joint trajectory controller.
4. Keep position and `time_from_start` data.
5. Validate linear interpolation between every resulting position sample.

With position-only trajectory points, the active ROS 2 spline controller uses
linear interpolation. This matches the executor's current interpolation model
and avoids unvalidated quintic Cartesian excursions.

Apply this first to all Cartesian operations, including lower, paint, lift, and
straight retreat. Do not alter ordinary joint-space travel trajectories in the
same change.

### Validation Changes

Extend Cartesian validation to report separate values for:

- Signed maximum inward canvas-normal deviation.
- Signed maximum outward canvas-normal deviation.
- Maximum tangential distance from the requested path.
- Maximum tool-orientation deviation.

Add a dedicated parameter such as:

```yaml
max_cartesian_normal_deviation_mm: 0.2
controller_sample_dt: 0.005
```

The final normal limit must remain configurable and be approved against the
spring and paper, but it must be substantially smaller than the existing
`2.0 mm` total Cartesian deviation limit.

### Tests

- Confirm Cartesian trajectories sent to the controller contain positions and timestamps but no derivatives.
- Confirm interpolated midpoint positions match the executor's validator.
- Confirm normal deviation is checked independently from tangential deviation.
- Confirm joint-space travel trajectories retain their original derivatives.
- Run all existing path, launch, collision, and painter tests.

### Acceptance Gate

- Controller reference normal deviation remains within `+/-0.20 mm` for every diagnostic path.
- No orientation or collision regression occurs.
- Direction, reversal, and curve trajectories complete in fake hardware.
- The change passes an above-paper real-arm test before contact is allowed.

## 7. Phase 2: Instrument and Correct ServoJ Timing

### Goal

Make the ros2_control command period, actual ServoJ call interval, and ServoJ
`t` parameter agree without blocking or silently overrunning the Aubo queue.

### Phase 2A: Instrumentation

Instrument the Aubo hardware interface to record or publish:

- Measured ros2_control period.
- ServoJ RPC call duration.
- ServoJ return code.
- Queue-full retry count.
- Consecutive missed or late update count.
- Minimum, mean, maximum, and percentile loop periods.

Logging must be throttled so diagnostics do not create additional timing load.
Unexpected return codes must no longer be silently discarded.

### Phase 2B: Matched Timing Trials

Evaluate matched update-rate and ServoJ-period pairs above the paper:

| Trial | Controller update rate | ServoJ `t` |
| --- | ---: | ---: |
| A | `125 Hz` | `0.008 s` |
| B | `200 Hz` | `0.005 s` |

Trial A is included because the driver repository already contains an Aubo i5
`125 Hz` update-rate profile. Trial B matches the current controller target and
the SDK's 5 ms example.

Do not test the two rates in one run. Restart the driver between trials and
record a separate bag for each. Keep path geometry, velocity scaling,
acceleration scaling, tool pose, and canvas pose unchanged.

The implementation should expose the ServoJ period as reviewed configuration
or derive it from one authoritative controller period. It must reject an
obvious configuration mismatch rather than silently using `0.01 s` with a
5 ms target loop.

### Queue-full Policy

The current unbounded blocking retry must be replaced only after instrumentation
shows how often it occurs. The selected policy must:

- Avoid blocking the complete controller loop for an unbounded duration.
- Preserve the newest safe position command.
- Surface repeated queue saturation as a hardware/control error.
- Avoid silently continuing a painting trajectory with stale setpoints.

### Acceptance Gate

- Actual update rate remains at least 95 percent of the configured rate.
- No queue-full event occurs during the diagnostic fixtures.
- No unexplained ServoJ error code occurs.
- Median common joint delay is below `30 ms` and the 95th percentile is below `50 ms`.
- Actual canvas-normal tracking remains within the provisional hover limit of `+/-0.25 mm`.

If neither timing pair meets the gate, stop contact testing and investigate the
RPC streaming architecture before tuning lookahead or gain.

## 8. Phase 3: Settled Endpoint Validation

### Goal

Eliminate false endpoint aborts without weakening the existing endpoint error
limit.

### Implementation

After MoveIt reports execution success:

1. Continue collecting fresh measured joint states.
2. Compute per-joint endpoint error and TCP position/orientation error for each sample.
3. Require measured joint velocity below a configured threshold.
4. Require all endpoint checks to pass for a configured number of consecutive samples or duration.
5. Fail only when the settling timeout expires.

Proposed initial parameters:

```yaml
endpoint_settle_timeout_s: 1.0
endpoint_settle_duration_s: 0.1
endpoint_settle_velocity_rad_s: 0.01
```

Keep these existing limits unchanged during the first implementation:

```yaml
max_execution_tip_error_mm: 1.0
max_execution_tip_orientation_error_deg: 1.0
```

Every timeout should log per-joint signed error, canvas X/Y/Z TCP error,
measured velocity, sample count, and elapsed settling time.

### Acceptance Gate

- Ten repeated hover executions complete without a false endpoint abort.
- A deliberately unreachable endpoint still fails after the timeout.
- No stale joint-state sample can satisfy the settling requirement.
- Retreat behavior remains safe when settling fails with possible pen contact.

## 9. Phase 4: Continuous Canvas-Normal Telemetry and Guarding

### Goal

Detect unsafe inward or outward motion during a stroke instead of checking only
after it finishes.

### Implementation

During every contact trajectory, compute at controller-state rate:

- Controller-reference TCP canvas X/Y/Z.
- Measured TCP canvas X/Y/Z.
- Actual-minus-reference canvas X/Y/Z.
- Estimated spring compression: `plane_bias_mm + actual_canvas_z_mm`.
- Per-joint reference-minus-actual error.
- Current canvas direction and speed.

Publish or record these values through a lightweight diagnostics topic and
include extrema in the command completion log.

Add warning and hard-stop thresholds only after hover data confirms the monitor
and the physical spring's safe range is reviewed. A provisional paper-contact
target for the current `1.0 mm` bias is:

```text
Estimated compression: 0.5 mm to 1.5 mm
Equivalent actual canvas Z: -0.5 mm to +0.5 mm
```

The hard-stop design must include a tested cancellation and straight-retreat
state machine. A cancellation that leaves the pen down without a controlled
retreat is not an acceptable safety implementation.

### Acceptance Gate

- Telemetry agrees with offline bag analysis within `0.05 mm`.
- Warning and cancellation logic is exercised with fake hardware.
- A simulated hard-limit violation cancels once and executes one bounded retreat.
- No callback or logging work reduces the control-loop rate.

## 10. Phase 5: Dynamics, Constraints, and Residual Mechanics

Complete this phase only after interpolation and ServoJ timing meet their gates.

### Dynamics

- Obtain Aubo i5 acceleration limits from verified manufacturer data.
- Enable real acceleration limits in `joint_limits.yaml`.
- Confirm velocity and acceleration scaling affect the generated timestamps.
- Re-run normal-deviation validation after retiming changes.

### Controller Constraints

- Add measured per-joint trajectory and goal tolerances.
- Select tolerances from stable hover data, not from the current faulty runs.
- Verify that a trajectory violation cancels safely during pen contact.

### Payload and Mechanics

- Verify claw, spring, and pen payload mass in the Aubo controller.
- Verify payload center of gravity.
- Check spring guide binding and holder flex after encoder-visible errors are reduced.
- Check paper and backing flatness with the arm stationary.

Residual physical compression changes with small encoder-visible error indicate
payload compensation, structural flex, holder motion, or backing deflection.

## 11. Hardware Verification Sequence

Use the generated fixtures in this order:

```text
output/arm_tracking_direction_test_paths.json
output/arm_tracking_reversal_test_paths.json
output/arm_tracking_curve_test_paths.json
output/sine_test_paths.json
```

### Stage 1: Software and fake hardware

- Run unit tests and package tests.
- Inspect RViz trajectories and collision geometry.
- Confirm controller-reference interpolation passes the signed normal limit.
- Confirm all diagnostic paths use the calibrated robot model.

### Stage 2: Real arm above paper

- Execute the same paths on a plane offset safely above the physical paper.
- Record controller state, joint state, executor logs, and ServoJ diagnostics.
- Compare rate, delay, joint error, and canvas-normal error against the July 22 baseline.
- Repeat each candidate ServoJ timing configuration with no other changes.

### Stage 3: Paper contact

Permit contact only when all previous gates pass. Use sacrificial paper, the
existing `1.0 mm` bias, and operator supervision. Run direction, reversal, and
compact curve tests before the sine path.

Paper-contact acceptance requires:

- No visible paper indentation or tearing.
- No gap or dotted line caused by contact loss.
- No spring approach to its mechanical travel limit.
- Estimated compression remains in the approved envelope.
- No endpoint or trajectory-limit abort.
- Repeatable results in both directions at low and high canvas Y.

## 12. Verification Commands

Run Python path tests from the RobRoss repository root:

```bash
python3 -m unittest discover Image_Process/mondrian/tests
```

Build and test the affected ROS packages from the workspace root:

```bash
colcon build --packages-select robross_painter aubo_ros2_driver
colcon test --packages-select robross_painter aubo_ros2_driver
colcon test-result --verbose
```

Record each hardware comparison:

```bash
ros2 bag record \
  /joint_trajectory_controller/controller_state \
  /joint_states \
  /rosout \
  /robross_markers
```

Also record the three source revisions and copy the effective runtime parameter
dump into the test notes.

## 13. Expected Files

Likely RobRoss changes:

```text
ros2/robross_painter/src/painting_executor.cpp
ros2/robross_painter/scripts/analyze_tracking_bag.py
ros2/robross_painter/config/hardware_a4.yaml
ros2/robross_painter/test/
ros2/robross_painter/README.md
docs/painting-executor-motion-translation.md
```

Likely Aubo driver changes in the sibling repository:

```text
../aubo_ros2_driver/aubo_ros2_driver/src/aubo_hardware_interface.cpp
../aubo_ros2_driver/aubo_ros2_driver/include/aubo_hardware_interface.h
../aubo_ros2_driver/aubo_ros2_driver/config/aubo_controllers.yaml
../aubo_ros2_driver/aubo_ros2_driver/test/
```

Changes spanning the two repositories must be reviewed and versioned together.
The painting path JSON schema does not need to change for this work.

## 14. Rollback and Comparison Rules

- Keep each phase in a separate commit or otherwise independently reversible change set.
- Preserve the July 22 bags as the baseline dataset.
- Keep generated path geometry unchanged across A/B comparisons.
- Revert the current phase if it worsens normal error, timing jitter, collision behavior, or retreat behavior.
- Do not retain a partial safety monitor that can cancel motion but cannot retreat safely.
- Do not approve contact based only on endpoint accuracy; continuous normal tracking must pass.

## 15. Completion Criteria

This remediation is complete when:

1. The controller reference stays within the approved canvas-normal limit.
2. The physical arm tracks that reference within the approved normal limit across the A4 test region.
3. Endpoint checks wait for verified settling and no longer fail nondeterministically.
4. ServoJ timing is matched, measured, and free of queue saturation during test paths.
5. Direction, reversal, compact curve, and sine tests draw continuously without tearing.
6. The approved settings, source revisions, and hardware procedure are documented and reproducible.
