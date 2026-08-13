# August 11-12 Hardware Remediation Plan

## Strategy

This plan closes the failure paths established in the
[August 11-12 root-cause findings](8-11-bag-root-cause-findings.md). Use defense
in depth: prevent wall-clock steps, make source-feedback health mandatory,
separate pre-execution failures from uncertain post-execution state, and require
the trajectory controller to reject invalid or materially divergent measured
state. Do not weaken the RTDE watchdog, MoveIt duration monitoring, collision
checking, endpoint checks, or retreat constraints to make a test pass.

The selected controller approach is a RobRoss-maintained fork of
`ros-controls/ros2_controllers`, patched on the ROS 2 Humble release line, built
in the workspace, and pinned in the vcstool manifest to an immutable commit SHA.
Do not patch `/opt/ros`, track a moving branch, or rely on an unrecorded Debian
package version. The selected clock approach is Chrony in slew-only mode: remove
all `makestep` directives, disable competing time-sync services, synchronize
before starting ROS, and never run `chronyc makestep` while the robot stack is
active.

## Phase 0 - Immediate Containment

- Preserve the six bags, logs, effective YAML, binary build IDs, and source
  diffs identified in the findings.
- Do not perform contact runs until Phases 1-5 and their non-hardware tests pass.
- Keep all seven painter conformance checks and endpoint settling enabled for
  qualification.
- Keep the RTDE watchdog at 50 ms and MoveIt duration monitoring enabled.
- Restore the checked-in hardware profile to a genuine all-checks-enabled,
  `dry_run: true` template before it is used as a new run starting point.

**Exit criterion:** the evidence baseline is immutable and no further contact
run can start from a profile that weakens the confirmed safety barriers.

## Phase 1 - Driver Health and RTDE Telemetry

- Add a typed, reliable, transient-local `/aubo_driver/health` heartbeat carrying
  starting/ready/faulted state, latched fault code and reason, writes-enabled
  state, source sequence, source age, requested/observed rates, and delivery
  statistics. Publish periodically and immediately after a fault transition.
- Capture `CLOCK_MONOTONIC` at RTDE callback entry. Parse into temporary values,
  require six finite positions and velocities, and commit the complete sample
  atomically. A malformed or exception-producing callback must not modify the
  last good state or refresh valid-source freshness.
- Record callback and valid-sample counts and ages, inter-arrival
  mean/p95/p99/max, counts over 10/20/50/100 ms, malformed vectors, parser
  exceptions, and bounded interval history preceding a fault.
- Parameterize the requested source rate as `rtde_state_rate`. The first
  instrumented run retained 500 Hz; the controlled comparison showed that the
  robot delivered approximately 200 Hz for either request, so the driver now
  defaults to 200 Hz. Keep 50 ms stale detection.
- Route RTDE stale/malformed/exception faults and every existing ServoJ
  write-disabling condition into one process-lifetime latch. Once faulted,
  invalidate velocity and fail both hardware reads and writes until process
  restart; lifecycle reactivation must not clear the latch.

**Exit criterion:** driver/message packages build; deterministic tests cover
statistics, threshold boundaries, ages, and reset behavior; malformed input
cannot become valid cached state; the health heartbeat exposes source progress
and the first latched fault; and a hardware soak can localize a new stall.

## Phase 2 - Painter Fail-Closed Behavior

- In the painter, treat non-finite position or velocity, stale source feedback,
  and a latched driver fault as hardware-health failures independent of all
  optional geometric and settling toggles.
- Add a driver health/diagnostic signal carrying latch state, source age, and
  fault reason. A newly published cached sample must not establish source
  freshness.
- After any health failure, cancel/stop the active goal, prohibit the next
  command, mark contact state uncertain, and permit only the existing bounded
  measured-state retreat when its prerequisites are satisfied.
- Keep fresh-state, bounds, collision, elbow/posture, guarded-joint, and contact
  checks mandatory. Disabling settling removes only the stationary dwell.

**Exit criterion:** unit and fake-hardware tests prove that every optional
conformance toggle can be off while `NaN`, stale, frozen-faulted, or explicitly
faulted feedback still aborts before another trajectory is planned.

## Phase 3 - Separate Motion Outcomes

- Replace the Cartesian boolean result with outcomes for planning/fraction
  failure, pre-execution validation failure, execution failure, and
  post-execution feedback/endpoint failure.
- Allow pen-up joint-space fallback only when Cartesian planning or validation
  fails before submission. Once execution starts, any failure must return to the
  abort/retreat state machine without automatic replanning.
- Log the numeric and symbolic `MoveItErrorCode`, controller action result, goal
  identifier, execution-started flag, and selected recovery path.

**Exit criterion:** tests prove fallback occurs for an eligible pre-execution
failure and never occurs after goal submission, timeout, cancellation, invalid
feedback, or endpoint rejection.

## Phase 4 - Pin and Harden the Trajectory Controller

- Fork the Humble `ros2_controllers` baseline matching the qualified binary,
  patch `joint_trajectory_controller` to abort rather than succeed when required
  measured position or velocity is non-finite, and add upstream-style tests.
- Pin the reviewed fork commit by full SHA in the workspace vcstool manifest.
  Build it before the Aubo driver and verify package prefixes/build IDs prove the
  source-built controller is loaded instead of `/opt/ros`.
- Configure finite per-joint trajectory and goal tolerances, finite stopped
  velocity tolerance, and bounded goal-time allowance. Derive values from stable
  hover evidence with explicit margin; do not tune them during contact.
- Preserve MoveIt execution-duration monitoring. Chrony removes wall-clock
  steps; the controller patch does not justify accepting a MoveIt timeout.

**Exit criterion:** controller tests show normal lag succeeds within the
qualified allowance, while frozen position, excessive lag, non-finite state,
and missed goal time return non-success. The manifest is immutable, the loaded
artifact matches its recorded build ID, and no unpatched binary can satisfy the
launch/build gate.

## Phase 5 - Stabilize the Hardware Host

- Install and enable Chrony; disable `systemd-timesyncd` and any other clock
  writer. Configure Chrony without `makestep`, including startup directives, so
  corrections during operation are slewed.
- Gate ROS startup on `chronyc tracking` and `chronyc waitsync`. Monitor
  realtime-minus-monotonic offset during every run and archive clock status.
- Grant the controller manager realtime priority, require and verify
  `SCHED_FIFO/50` on physical hardware, and use the performance power profile
  during qualification.
- Do not disable MoveIt execution-duration monitoring.

**Exit criterion:** after reboot, Chrony is the only synchronizer, synchronization
passes before ROS, the control thread is verified FIFO/50, and a no-motion soak
has no backward clock event or realtime scheduling warning.

## Phase 6 - Runtime Provenance

- Write an atomic qualification manifest before motion containing executable and
  plugin paths, ELF build IDs and hashes, source revisions and dirty status,
  calibration/canvas/path/URDF/SRDF hashes, effective parameters, robot type,
  joint orders, tool/canvas transforms, controller rate, ServoJ `t`, RTDE rate
  and watchdog, conformance switches, realtime policy, and Chrony status.
- Add `qualification_run` and `provenance_dir` launch arguments. Qualification
  mode must fail before motion if the evidence directory is not writable or a
  mandatory input/check is absent.
- Collect only an explicit allowlist; do not archive environments, credentials,
  or unrelated process and journal data.

**Exit criterion:** every accepted run can prove exactly which source, binaries,
models, parameters, calibration, host policy, and inputs produced it.

## Hardware Qualification Gates

Run each gate with all checks enabled, a fresh evidence directory, immediate
e-stop access, and the exact capture requirements in the
[hardware run guide](hardware_run_guide.md). Failure blocks later gates and
requires diagnosis plus repetition from the earliest affected gate.

1. **Static/fault gate:** build and test all affected packages; verify lifecycle
   restart, provenance, Chrony, FIFO, diagnostics, and injected stale/`NaN`/
   frozen-state controller failures without commanding motion.
2. **Stationary soak gate:** remain connected longer than the longest prior
   time-to-fault; require valid finite state, complete RTDE statistics, no clock
   step, no queue-full/non-OK ServoJ result, and no unexplained latch.
3. **Hover repetition gate:** execute the complete artwork at safe hover height
   at least three times with the qualified 125 Hz/0.008 s pair. Require every
   action and endpoint check to pass within the frozen tolerances.
4. **Deliberate-fault hover gate:** inject or simulate stale, frozen, and
   non-finite feedback. Require controller and painter non-success, no subsequent
   command, and only a permitted bounded retreat.
5. **Single-contact gate:** draw one short straight line on sacrificial paper;
   inspect tracking, spring travel, retreat, and the complete evidence set.
6. **Extended-contact gate:** progress through curves/corners and then one full
   artwork. Stop on any timing, health, controller, endpoint, contact, or
   provenance violation.

## Required Test Matrix

| Layer | Required proof |
| --- | --- |
| Painter unit | Health checks remain mandatory with every optional toggle off; stale, cached-faulted, and non-finite state stop sequencing. |
| Painter unit/integration | Cartesian fallback is pre-execution only; MoveIt timeout and endpoint rejection enter fail-closed recovery. |
| Driver unit | Monotonic RTDE accounting, rate/percentile/threshold counters, fault snapshot, malformed data, and latch reset are correct. |
| Controller fork | Finite nominal state succeeds; frozen, divergent, non-finite, stopped-velocity, and goal-time violations fail the action. |
| Fake hardware | Goal cancellation, no-next-command invariant, bounded retreat eligibility, and startup provenance are end-to-end. |
| Host | Chrony slew-only configuration, single synchronizer, wait-for-sync gate, monotonic clock monitor, FIFO policy, and performance profile persist after reboot. |
| Hardware | Stationary soak, three full hover repetitions, deliberate fault response, short contact, and full contact progression pass in order. |

## Sequencing and Acceptance

Phase 0 precedes all implementation. Phase 1 defines the health contract consumed
by Phase 2. Phase 3 may proceed alongside Phase 2 after that contract is stable.
Phase 4 depends on the baseline fault fixtures; Phases 5 and 6 can proceed in
parallel with code work but cannot waive any software gate. Hardware qualification
cannot start until Phases 1-6 and all non-hardware tests pass.
Any change to clock policy, driver timing/rate/watchdog, controller fork or
tolerances, calibration, tool, or motion profile invalidates downstream hardware
qualification.

Remediation is accepted only when all hardware qualification gates pass with:

- no backward wall-clock event, MoveIt execution timeout, RTDE stale latch,
  non-finite feedback, queue-full/non-OK ServoJ result, unexplained state gap, or
  controller false success during nominal runs;
- deliberate frozen, divergent, stale, and non-finite feedback always producing
  non-success before another command, without post-execution joint fallback;
- three repeatable complete hover runs and the staged contact runs remaining
  within the frozen controller, endpoint, contact, and retreat limits;
- complete bags, RTDE/ServoJ telemetry, Chrony/FIFO evidence, effective config
  hashes, source revisions/dirty patches, and loaded binary build IDs for every
  accepted run; and
- independent review confirming that no safety monitor or threshold was disabled
  or relaxed merely to obtain a pass.
