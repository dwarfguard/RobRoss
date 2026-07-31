# Aubo ROS 2 Driver Telemetry Code Review

**Status:** Second follow-up complete; not approved for formal or unattended hardware testing

**Reviewed:** 2026-07-29; follow-up reviews 2026-07-30

**Scope:** Combined delta from `origin/robross-fixes` in
`src/aubo_ros2_driver`: local commit `77c2d5b` plus six modified driver files
on branch `robross-fixes`

## Resolved Issues

Earlier review findings are summarized here rather than retained in full:

| Area | Resolution |
| --- | --- |
| Real-time telemetry handoff | Replaced the shared sample mutex with a bounded SPSC queue; full queues drop and count samples instead of blocking `write()`. |
| Queue-full behavior | Replaced unbounded ServoJ retries with a bounded policy and persistent-drop fault latch. |
| SDK-call instrumentation | Hoisted interface lookup outside the RPC timer, added a steady-clock call timestamp, and now records exception cycles explicitly. |
| CSV completeness | Added sequence numbers and a terminal summary; sample drops and runtime sink failures prevent `.partial` from being finalized. |
| Normal reactivation | Clean deactivation stops and joins the worker so the next activation opens a fresh sink. |
| Periodic diagnostics | Moved percentile sorting, formatting, and `/rosout` publication to an off-RT worker through a bounded report queue. |

The current findings below describe the remaining limitations in these areas.

## Current Findings

### P1: ServoJ ROS timestamp is still not the SDK-call timestamp

The code derives `t_call_ros_s` from the control-loop `time` argument and a
`steady_clock` sample taken later at `write()` entry:

- `aubo_ros2_driver/src/aubo_hardware_interface.cpp:366-404`

ROS 2 control defines `time` as the start of the control-loop iteration. It is
not paired with the later steady-clock sample, so the mapping omits time spent
in hardware read and controller update before `write()`. This makes the derived
call timestamp early by a variable amount and biases command-to-feedback delay
measurements.

Capture the ROS/feedback clock and steady clock together near the SDK call
boundary. Until then, use the CSV steady timestamp only for call-cadence
analysis, not formal phase-delay acceptance.

### P1: Activation failure can leave telemetry active

`on_activate()` opens the telemetry sink and starts the worker before the
failure-prone robot initialization in `OnActive()`:

- `aubo_ros2_driver/src/aubo_hardware_interface.cpp:209-229`
- `aubo_ros2_driver/src/aubo_hardware_interface.cpp:706-780`

If SDK initialization throws, no scoped cleanup stops the worker or closes the
sink. A later activation can reopen the stream while the old worker may still
be using it.

Make activation transactional and clean up every failure path. Until fixed,
restart the complete driver after an activation failure instead of retrying in
the same process.

### P2: Stale-output removal is unchecked

The previous final CSV and legacy invalid marker are removed without checking
the results:

- `aubo_ros2_driver/src/aubo_hardware_interface.cpp:729-759`
- `aubo_ros2_driver/src/aubo_hardware_interface.cpp:947-958`

If removal fails, activation still succeeds and stale output can remain visible
while final rename fails. Treat removal errors other than a missing file as
activation failures.

### P2: Raw report queue copies indeterminate array elements

`ServoTimingStats::RawReport` leaves unused fixed-array elements uninitialized.
`write()` fills only the active prefixes and then pushes the entire object
through Boost's SPSC queue:

- `aubo_ros2_driver/include/servo_timing_stats.h:143-168`
- `aubo_ros2_driver/src/aubo_hardware_interface.cpp:495-501`

The queue copy-constructs the full report, including indeterminate `double`
elements beyond `period_count` and `rpc_count`. This is undefined behavior on
the control thread and occurs even when CSV telemetry is disabled.

Initialize both arrays, or otherwise ensure the complete copied object is
initialized, before extended robot motion testing.

## Hardware-Test Assessment

The current state is not ready for formal Phase 2B acceptance, unattended
motion, paper contact, or painting tests.

A limited, supervised free-space smoke test may be considered as exploratory
work, preferably after fixing `RawReport` initialization. If testing proceeds:

- Use reduced speed with an operator at an accessible emergency stop.
- Do not make paper contact or run a painting path.
- Restart the driver after any activation failure.
- Do not use `t_call_ros_s` for command-to-feedback delay conclusions.
- Treat queue-full events, timing faults, `.partial` output, abnormal shutdown,
  or failed final rename as an invalid run.

Formal Phase 2B trials require all four current findings to be resolved and the
failure and lifecycle paths to be tested.

## Verification

- `git diff --check origin/robross-fixes` passed in the driver repository.
- `colcon build --packages-select aubo_ros2_driver --cmake-args
  -DBUILD_TESTING=ON` passed.
- Driver tests reported 91 tests with zero errors, failures, or skips.
- The focused ServoJ gtest result contains 19 passing tests.
- Tests do not cover activation failure, worker cleanup, sink removal or
  finalization failure, ROS/steady clock mapping, or the full `RawReport` queue
  copy under memory instrumentation.
