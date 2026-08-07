# Aubo Painting Current Status (2026-07-31)

**Status:** Active hardware engineering record
**Last reviewed:** 2026-08-06
**Scope:** Aubo i5 pen-on-paper execution, tracking evidence, and remaining work
**Authority:** This file governs hardware decisions and evidence interpretation;
the [hardware run guide](hardware_run_guide.md) governs the per-session procedure.

## Current Decisions

- Use the matched `125 Hz` controller and ServoJ `t=0.008 s` pair for hardware
  motion. It completed stationary, hover, and contact operation without queue
  saturation, and is now the driver default (a bare `aubo_control.launch.py`
  comes up on it).
- The `200 Hz / 0.005 s` pair is disqualified — not a launch option and not for
  painting. Valid stationary evidence showed approximately 20.4 percent
  queue-full drops.
- The current hover diagnostic set is direction, reversal, and alternating-curve
  motion. `sine_test_paths.json` is obsolete and intentionally removed.
- Deliberate fixture repetitions are valid repeatability evidence. Record the
  intended repeat count and report each repeat separately as well as in aggregate.
- A supervised full contact artwork completed successfully on July 31. This is
  evidence for the supervised Demo v1 workflow, not approval for unattended
  operation.
- Report temporal lag separately from geometric path error. Do not interpret a
  simultaneous reference-versus-feedback offset as geometric deviation without
  also evaluating the time-aligned path.

## Implementation Status

| Area | Current status |
| --- | --- |
| Cartesian interpolation | Complete: Cartesian trajectories are resampled, stripped to position-only points, and checked for signed normal, tangential, and orientation deviation before execution. |
| ServoJ parsing and timing | Complete for the selected pair: `servoj_time` is validated and actual cadence is measured. A sustained large mismatch outside the driver's latch band faults; milder mismatches warn. One pre-activation authority for controller rate, ServoJ `t`, and Cartesian `controller_sample_dt` remains open. |
| Queue-full handling | Complete for supervised use: the real-time loop does not sleep indefinitely, retries are bounded, drops are measured, the newest safe command is retained instead of replaying stale setpoints, and persistent saturation latches a fault. |
| Full-rate command telemetry | Complete: one row per enabled write cycle records the command, cycle and final SDK-call timestamps, ServoJ `t`, final RPC duration/result, retry count, drop, and exception status. Inter-call cadence is derived from timestamps. |
| Lifecycle finalization | Complete for the demonstrated normal path: clean deactivation finalizes telemetry and supports reactivation. Targeted failure-injection evidence remains open. |
| Real-time scheduling | Implemented: the control loop requests FIFO scheduling and reports when permission is unavailable. The July 31 cadence was stable. |
| Offline bag analysis | Partial: useful for segmentation and geometry, but its historical global tracking screen and controller-state delay aggregation are not current acceptance criteria. |
| Endpoint settling | Open: execution currently checks one fresh post-execution sample rather than a settled sample window. |
| Continuous contact guarding | Open: there is no online spring-compression warning, cancellation, and straight-retreat state machine. |
| Dynamics and mechanics | Open: verified acceleration limits, controller tolerances, payload/CoG, and formal spring-envelope validation remain future work. |

## July 31 Evidence

### Stationary 125 Hz lifecycle

Evidence: `/home/robross/robross_hardware_20260731_142744_125Hz`

- Two completed lifecycle activations in one driver process.
- 7,202 and 1,832 contiguous full-rate rows.
- `servoj_time=0.008`, all return codes `0`, and no queue-full events,
  telemetry drops, or exceptions.
- Commands remained exactly stationary in both captures.

### Repeated hover motion

Evidence: `/home/robross/robross_hardware_20260731_150918`

- Direction, reversal, and alternating-curve fixtures completed three deliberate
  repetitions.
- Full-rate trail: 45,485 contiguous calls at approximately `125.005 Hz`.
- Mean/p95/p99 call intervals: approximately `8.000/8.040/8.073 ms`.
- Absolute full-rate maximum: approximately `11.079 ms`; aggregate diagnostics
  counted three isolated late cycles. The report-boundary maximum was
  approximately `8.120 ms`, and no sustained timing fault occurred.
- No queue-full events, non-OK return codes, drops, or exceptions.
- Paint-path mean absolute canvas-normal error was `0.103-0.111 mm` per repeat;
  p95 was `0.195-0.203 mm`.
- The three repeats measured the same temporal delay distribution: approximately
  `96/128 ms` median/p95 with the controller-state analyzer.

### Contact artwork

Evidence: `/home/robross/robross_aubo_ws/ros2_bag_robot_7_31`

- The bag is structurally valid and contains 171,568 messages over 703.7 seconds.
- `robot_path.json` completed all 1,466 commands, including 366 paint strokes.
- The operator judged the resulting drawing acceptable.
- Aggregate ServoJ diagnostics reported no queue-full events, non-OK return
  codes, exceptions, or late cycles. Four startup-delivered windows predate bag
  recording; 88,000 reported cycles fall on the bag timeline, including 80,750
  complete windows during command execution.
- Paint-path absolute canvas-normal error was approximately `0.142 mm` mean,
  `0.346 mm` p95, `0.413 mm` p99, and `0.629 mm` maximum.
- Approximately 99.72 percent of paint samples were within `0.5 mm` normal error.
- Median segment-average estimated spring compression was approximately
  `0.96 mm`.

The bag does not measure pen force and therefore cannot prove a hard contact-force
limit or authorize unattended use.

### Evidence provenance and profile scope

The July 31 contact result is successful outcome evidence, not reproducible
qualification of the current checked-in hardware template or exact source
binary. The session recorded modified driver source and untracked fixture/path
files, but it did not preserve the corresponding driver patch, untracked-file
archive, or tested-binary hash. The contact bag also lacks `servoj_config` and a
contiguous full-rate ServoJ CSV, so contact-run timing qualification is
incomplete under the current evidence rules.

The preserved contact profile also differs from the current template:

| Parameter | July 31 contact source | Current checked-in template |
| --- | ---: | ---: |
| Velocity/acceleration scaling | `0.025 / 0.025` | `0.1 / 0.1` |
| `eef_step_m` | `0.0002` | `0.005` |
| Normal-deviation limit | `0.5 mm` | `0.2 mm` |
| Endpoint-position limit | `2.0 mm` | `1.0 mm` |
| Cartesian sample interval | Legacy `totg_resample_dt: 0.02`; no `controller_sample_dt` recorded | `0.008 s` |

Treat the checked-in file as a dry-run template. A current measured profile must
complete the staged qualification in the hardware run guide before contact.

## Delay Diagnosis

The separate full-rate hover trail localizes the observed delay:

| Boundary | Result |
| --- | ---: |
| Controller reference to ServoJ command | Effectively zero lag; approximately `0.00038 deg` RMS at zero shift |
| ServoJ command to measured joint feedback | Approximately `88 ms` median and `120 ms` p95 |

The delay is downstream of the ROS trajectory controller and the SDK call
boundary. The remaining region includes Aubo internal ServoJ interpolation or
smoothing, physical joint response, and RTDE feedback age. Queue saturation is
not the cause at 125 Hz.

Most simultaneous tangential error is temporal rather than geometric. Applying
the hover-derived approximately `88 ms` alignment to the contact bag changes
tangential error
from approximately `2.00/6.25/7.84 mm` mean/p95/max to
`0.18/0.36/0.54 mm`. Keep latency as a diagnostic and evaluate corners and
endpoints, but do not use the old `<30/<50 ms` median/p95 target as an automatic
contact blocker without a demonstrated quality or safety relationship.

## Current Analysis Rules

- Separate `move_to`, `lower_tool`, `paint_path`, and `lift_tool` results.
- Do not use a maximum normal error pooled across all command types as a painting
  gate. Approach and retreat intentionally move along the canvas normal.
- For painting, report signed normal distribution, absolute mean, p95, p99,
  maximum, and estimated compression.
- Report raw simultaneous tangential error and time-aligned geometric error.
- Report each deliberate repetition independently before aggregating it.
- Use the full-rate ServoJ command trail for latency analysis. The approximately
  `62.5 Hz` controller-state stream is useful for geometry but is too coarse to
  be the sole timing authority.
- Treat the analyzer's `Historical global tracking screen` as a historical
  diagnostic until these rules are implemented in code.

The analyzer implementation is not complete until it consumes a contiguous
full-rate ServoJ command trail, correlates it with measured feedback, separates
command types and deliberate repetitions, and reports simultaneous and
time-aligned metrics. Missing or incomplete full-rate evidence must produce an
incomplete result rather than silently falling back to controller-state timing.

## Remaining Engineering Work

### Configuration authority

Controller `update_rate`, ServoJ `t`, and Cartesian `controller_sample_dt` are
still configured independently. Replace them with one authoritative timing
profile, or reject a mismatch before activation. The existing runtime guard
faults only after a large mismatch remains outside its latch band; warning-only
mismatches are not configuration approval.

### Endpoint settling

Replace the single post-execution sample with a timeout-bounded window of fresh
samples. A pass must require consecutive samples satisfying per-joint error,
TCP position and orientation error, and low measured velocity. Reject stale
samples and log signed errors, velocity, sample count, and elapsed time. Verify
both the passing path and a timeout/fault path; a contact-time failure must use
the bounded straight retreat. Do not raise endpoint limits merely to suppress
an abort.

### Continuous contact guard

Monitor reference and measured TCP in canvas coordinates, actual-minus-reference
error, per-joint error, direction and speed, and estimated spring compression.
Canvas `+Z` is into the paper; estimated compression is `plane_bias_mm` plus
measured canvas Z. Thresholds require a physically reviewed spring envelope.
Cancellation must trigger exactly one bounded straight retreat and must never
leave the pen down. Verification requires fake-hardware fault injection,
agreement between online and offline calculations, and proof that monitoring
does not degrade control cadence.

The current pen spring has `3.8 mm` total mechanical travel. That is a hard
bottom-out fact, not an approved operating envelope; contact limits require a
reviewed margin below it.

### Dynamics, controller, and mechanics

- Obtain manufacturer-supported acceleration limits, enable
  `has_acceleration_limits`, verify that velocity and acceleration scaling
  change trajectory timestamps, and repeat post-retiming path-deviation
  validation.
- Select trajectory and goal tolerances from stable hover evidence, then test
  their cancellation behavior during possible contact.
- Verify payload mass and center of gravity, spring-guide binding, holder flex,
  and stationary paper/backing flatness. Residual compression changes may come
  from payload compensation, structural flex, holder motion, or backing
  deflection rather than encoder-visible path error.
- The driver still requests the RTDE state topic at 500 Hz. Reducing that request
  to a rate the controller actually delivers is an optional latency/jitter
  investigation, not a blocker for the demonstrated 125 Hz supervised mode;
  any change requires renewed qualification.

### Verification gaps

The available July 31 evidence covers successful activation, deactivation,
reactivation, and telemetry finalization. It does not replace targeted tests for
activation cleanup, telemetry-worker shutdown, stale-output removal failure,
sink/final-rename failure, hardware validation of ROS-to-steady call timestamp
mapping, or memory-instrumented coverage of complete report-queue copies. Close
these gaps before unattended operation.

## Safety Invariants

- Preserve collision checking, the canvas backing plane, calibrated claw and
  tool geometry, the elbow-family policy, guarded-joint limits, Cartesian jump
  detection, and above-paper validation after every behavioral motion change.
- Do not add global, position-dependent, or direction-dependent Z compensation
  to hide tracking behavior. Do not weaken collision, path, posture, or endpoint
  limits merely to make a rejected motion execute.
- During supervised contact, stop for paper indentation or tearing, gaps or
  contact loss, spring travel approaching its mechanical limit, an endpoint or
  trajectory abort, nonrepeatable directional behavior, unexpected geometry,
  or unstable wrist motion toward the paper.
- A contact-state abort permits one bounded straight retreat only. Never start
  an unconstrained or joint-space retreat while the pen may still touch paper.

## Evidence Capture

For every reviewed hardware session:

1. Start the bag before the driver so it captures `servoj_config`.
2. Record `/joint_trajectory_controller/controller_state`, `/joint_states`,
   `/robot_description`, `/rosout`, `/tf`, `/tf_static`, `/robross_markers`, and
   `/parameter_events`.
3. Enable a new full-rate ServoJ CSV and require a contiguous sequence with a
   `status=complete dropped=0` footer.
4. Record all three source revisions. If a worktree is intentionally dirty,
   preserve `git diff --binary HEAD` plus an archive of untracked files and hash
   both; a commit hash alone does not describe the tested binary.
5. Hash the exact path JSON, calibration YAML, canvas YAML, controller profile,
   and generated patch evidence before motion.
6. Preserve the launch command and intended repeat count. Do not race a
   short-lived executor with a manual cross-terminal parameter dump.
7. Invalidate qualification evidence after any queue-full event, non-OK ServoJ
   return, exception, timing fault, incomplete `.partial` output, telemetry drop,
   or failed finalization. Preserve the evidence and investigate it.
8. Change one timing or interpolation variable per recorded comparison; keep
   geometry and scaling fixed. Roll back a change that worsens tracking, jitter,
   collision behavior, or retreat behavior.
9. Review driver and description changes together. Record both revisions and
   ensure the referenced description commit is available before publishing a
   parent driver revision.

The exact July 31 contact path currently hashes to:

```text
039186e7911bb9d76eaeecf0bc849388bd8479f172a385de0225069393a2626a  output/robot_path.json
```

## Operating Modes

### Supervised Demo v1

The July 31 evidence supports continued supervised pen drawing with the selected
125 Hz pair when the applicable gates in the
[hardware run guide](hardware_run_guide.md) pass, the exact inputs are preserved,
the operator has immediate e-stop access, and the paper, spring, and drawing are
inspected during the run. Stop immediately for any condition listed under
Safety Invariants.

### Unattended Operation

Unattended contact remains unapproved. It requires, at minimum:

- settled endpoint validation;
- an approved spring-compression envelope;
- continuous contact telemetry;
- tested cancellation and straight-retreat behavior;
- direct force sensing with a tested hard-force limit;
- verified controller tolerances and robot acceleration limits;
- payload, center-of-gravity, and residual mechanical review;
- targeted lifecycle, telemetry-finalization, timestamp-mapping, and
  memory-instrumented failure-path verification listed above.
