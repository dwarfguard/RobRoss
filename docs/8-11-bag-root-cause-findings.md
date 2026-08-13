# Root-Cause Findings: 2026-08-11 and 2026-08-12 Hardware Bags

**Date investigated and updated:** 2026-08-12
**Machine investigated:** the same machine that ran the hardware tests
**Recordings:** `8-11_Bag`, `8-11_Bag_2_no_checks`,
`8-11_Bag_3_no_checks`, `8-12_Robot_1_noCheck`, `8-12_Robot_2`, and
`8-12_Robot_3`

This report supersedes the preliminary causal explanations in
`8-11-bag-issue-summary.md`. It combines all six bags with the machine's
persistent ROS logs, launch logs, system journal, current source, git history,
and build artifacts. No hardware was moved during the investigation.

## Executive Summary

The three August 11 runs do not share one root cause:

| Bag | Confirmed primary cause | Important secondary defect |
| --- | --- | --- |
| `8-11_Bag` | The Aubo RTDE state watchdog faulted, froze the reported position, invalidated velocity, and disabled ServoJ writes. | A Cartesian endpoint-validation failure was incorrectly treated as Cartesian planning infeasibility and retried in joint space. |
| `8-11_Bag_2_no_checks` | The same RTDE watchdog fault occurred around command 780. Subsequent commands were accepted while the robot state remained frozen. | With settling and endpoint checks disabled, the painter accepted a newly republished cached position as fresh even though velocity was `NaN`. |
| `8-11_Bag_3_no_checks` | The machine's wall clock changed during trajectory execution. MoveIt timed out while the controller completed the goal successfully. | MoveIt and ROS timestamps based on system time made clock discontinuities look like feedback gaps and caused misleading event ordering. |

The August 12 runs independently reproduced both primary failure modes:

| Bag | Outcome | Confirmed primary cause |
| --- | --- | --- |
| `8-12_Robot_1_noCheck` | RTDE fault followed by continued command acceptance; the operator interrupted at command 203. | The state watchdog faulted while every optional conformance check was disabled. Cached positions continued to look fresh, and measured tip error grew to 66.835 mm. |
| `8-12_Robot_2` | The painter aborted at command 2203 and refused an autonomous retreat from unknown state. | The RTDE watchdog faulted. Enabled endpoint settling rejected non-finite velocity and the frozen endpoint instead of allowing execution to continue. |
| `8-12_Robot_3` | The painter aborted at command 1556 and successfully retreated to hover. | A backward wall-clock adjustment caused MoveIt to time out while the controller remained healthy and completed successfully about 60 ms later. |

The reported 36 mm endpoint error in bag 2 was not caused by joint ordering,
the wrong trajectory endpoint, or an inconsistent tool transform. It was the
difference between a continuing commanded trajectory and a robot state that had
stopped changing after the RTDE fault.

The reported 653 ms feedback gap in the August 11 bag 3 was not evidence of a rosbag,
publisher, controller, or network stall. Three apparent long gaps correspond to
three system-clock discontinuities. The final failure was a MoveIt execution
wait timeout caused by a backward clock adjustment, not a controller execution
failure. August 12 run 3 repeated the same pattern, including three backward
timestamp jumps and a controller goal that succeeded after MoveIt timed out.

## Evidence Sources

The investigation used:

- The six SQLite rosbag2 recordings and their embedded message headers.
- `/joint_states`, `/joint_trajectory_controller/controller_state`, `/rosout`,
  `/tf`, `/robot_description`, `/parameter_events`, and `/robross_markers`.
- Persistent logs under `/home/robross/.ros/log`.
- The system journal for the run intervals.
- The installed and build-tree `painting_executor` binaries.
- RobRoss source at git revision `e825253`, including uncommitted conformance
  toggle changes used by the August 11 bags 2 and 3 and all August 12 runs.
- The Aubo driver source in the neighboring `aubo_ros2_driver` repository.

The installed painter used by the August 11 bags 2 and 3 and all three August
12 runs was built on 2026-08-11 at 18:28. Its log messages uniquely match the
uncommitted per-check implementation. August 11 bag 1 predates that build and
used the earlier endpoint-settling implementation.

All runs launched the installed executable:

```text
/home/robross/robross_aubo_ws/install/robross_painter/lib/robross_painter/painting_executor
```

The launched painters layered `/home/robross/hardware_a4.yaml` and
`/home/robross/canvas_calibration.yaml` over the launch-generated robot model
parameters. The August 12 run 2 and run 3 directories also contain archived
copies of `hardware_a4.yaml`. Both copies enable all seven conformance checks;
the only difference between them is an endpoint settle dwell of 25 ms for run 2
and 35 ms for run 3.

## Finding 1: Bags 1 and 2 Hit the RTDE Watchdog

### Confirmed behavior

Bag 1's driver logged the fault at Unix time `1786442455.039`:

```text
RTDE joint-state feedback is stale; latching a hardware fault,
invalidating velocity feedback, and refusing further ServoJ writes
until the driver is restarted
```

Evidence:

```text
/home/robross/.ros/log/aubo_ros2_control_node_1470317_1786440979672.log:9625
```

Bag 2 logged the same fault at Unix time `1786446698.376`, around command 780:

```text
/home/robross/.ros/log/aubo_ros2_control_node_1492858_1786446447328.log:2483
```

After each fault:

- all six published joint positions remained unchanged;
- all six published velocities became `NaN`;
- `/joint_states` continued to be published near 125 Hz;
- the driver refused subsequent ServoJ writes;
- the trajectory controller continued accepting goals and reporting success.

This behavior follows directly from the driver implementation:

- `aubo_hardware_interface.cpp:503-520` latches the fault, disables writes, and
  fills the copied velocity state with `NaN`;
- `aubo_hardware_interface.cpp:708-730` continues exposing the cached position;
- `aubo_hardware_interface.cpp:1423-1433` refreshes the watchdog only in the
  source RTDE callback.

This is fail-closed at the ServoJ write boundary, but the resulting ROS state
stream is only partially fail-closed: the invalid velocity communicates the
fault, while the repeatedly published cached position can look fresh to a
consumer that ignores velocity.

### Why the RTDE watchdog trips

The immediate trigger is known: no source RTDE state callback arrived within
the configured 50 ms maximum age. The lower-level reason for those packet
interruptions is not yet proven.

Relevant evidence and machine conditions:

- The SDK is asked for a 500 Hz RTDE state topic, but its own five-second report
  consistently shows approximately 1,000 packets, or about 200 Hz.
- The watchdog permits one maximum age of 50 ms and latches permanently on the
  first violation.
- The same fault occurred in other runs on August 7 and twice again on August
  12, so it is recurrent and independent of endpoint checks and rosbag.
- ServoJ call timing remained nominal near 125 Hz before the faults.
- The driver could not enable FIFO realtime scheduling on every inspected run.
- The current user has no realtime priority allowance (`rtprio` is zero).
- The machine uses an ASIX AX88179A USB Ethernet adapter with the `cdc_ncm`
  driver for the direct robot connection.
- USB runtime autosuspend for that adapter is disabled (`power/control` is
  `on`), and current interface counters show no RX/TX errors or drops.
- CPU frequency policy is `schedutil`, and the system power profile is
  `balanced`.

There is not enough source inter-arrival telemetry to distinguish among an SDK
callback stall, robot RTDE delivery pause, USB/network scheduling latency, or
host scheduling latency. The watchdog correctly detects an absence of fresh
source data, but its current log does not record the actual age, preceding
inter-arrival distribution, or packet sequence information needed to localize
the pause.

## Finding 2: Bag 1's Second Attempt Was an Executor Defect

Bag 1 reported both Cartesian and joint-space settling failures for command
2957. They used the same frozen measured endpoint because no ServoJ writes were
possible after the RTDE fault.

`doMoveTo()` currently interprets every `moveCartesian()` false return as a
straight-line planning failure and falls back to joint-space planning:

```text
ros2/robross_painter/src/painting_executor.cpp:914-943
```

However, `moveCartesian()` also returns false after all of the following have
already happened:

1. Cartesian planning succeeded.
2. Trajectory validation succeeded.
3. The trajectory was submitted for execution.
4. MoveIt reported execution success.
5. Post-execution endpoint settling failed.

Relevant path:

```text
ros2/robross_painter/src/painting_executor.cpp:2271-2343
ros2/robross_painter/src/painting_executor.cpp:2651-2758
```

A post-execution validation failure means contact or endpoint state is
uncertain. It must not enter a planning fallback. The joint-space retry was not
the cause of the original RTDE failure, but it produced the confusing second
settling timeout and could be unsafe in other failure modes.

## Finding 3: Bag 2's 36 mm Error Was Real Relative to the Command

The RTDE fault froze measured joint position around command 780. Commands
continued through command 867. As requested endpoints moved farther from the
frozen pose, the reported error grew naturally:

| Command | Maximum joint error | TCP error |
| ---: | ---: | ---: |
| 856 | 2.706 deg | 28.762 mm |
| 860 | 2.985 deg | 32.088 mm |
| 864 | 3.026 deg | 35.243 mm |
| 867 | 2.944 deg | 36.049 mm |

The controller's desired-versus-actual joint data independently reproduces the
2.944 degree final joint error. This rules out a painter-only formatting or
joint-index error.

The painter correctly maps state by joint name even though `/joint_states` and
the controller use different array orders:

```text
painting_executor.cpp:1548-1555
painting_executor.cpp:1718-1728
painting_executor.cpp:1764-1771
```

Endpoint calculations use the final returned trajectory point and consistent
inverse tool transforms:

```text
painting_executor.cpp:892-909
painting_executor.cpp:1750-1795
```

The runtime tool offset recorded in the bags matches the expected profile:

```text
tool_offset_xyz: [0.001208, -0.06034, 0.090753]
tool_offset_rpy: [0.0, 0.0, 0.0]
```

### Why the painter continued

With endpoint settling disabled, `executeTrajectory()` calls
`refreshTrackedState()` for one fresh post-completion sample. Freshness is based
on local ROS message receipt and a local sequence number. The driver continued
publishing new messages containing its cached position, so each publication
looked fresh even though the source RTDE state was no longer fresh.

Relevant code:

```text
painting_executor.cpp:361-395
painting_executor.cpp:1563-1591
painting_executor.cpp:2303-2343
```

All optional endpoint errors were disabled, and the frozen state remained
within hard bounds, elbow policy, and collision constraints. Therefore the
one-shot path logged the growing error but deliberately accepted it.

The drawing looking precise does not contradict this result. RViz stroke
markers are generated from requested canvas geometry, not reconstructed
measured TCP motion. Physical marks made before command 780 can also remain
precise while later requested segments are not executed.

## Finding 4: Bag 3 Was a System-Clock Failure

### The apparent feedback gaps

Bag 3 contains three apparent simultaneous gaps:

| Approximate gap | `/joint_states` | Controller state |
| --- | ---: | ---: |
| 1 | 597.7 ms | 605.8 ms |
| 2 | 652.5 ms | 652.7 ms |
| 3 | 581.4 ms | 589.5 ms |

Both bag timestamps and embedded header stamps show the gaps because both are
derived from the host system clock. Comparing those two clocks therefore cannot
detect a host wall-clock discontinuity.

The system journal contains repeated:

```text
systemd-resolved: Clock change detected. Flushing caches.
```

During the same run, `robot_state_publisher` detected three backward time
movements:

```text
/home/robross/.ros/log/robot_state_publisher_1495164_1786446800464.log:10-12
```

There are exactly three apparent long feedback gaps and three backward-time
warnings. The gap intervals align with the host clock-change sequence. They are
timestamp holes created by clock correction, not evidence that both feedback
publishers or rosbag stopped running.

### The final execution failure

At command 2496, MoveIt accepted and sent the trajectory. The wall clock then
moved backward while MoveIt was waiting for completion. MoveIt logged:

```text
waitForExecution timed out
Controller handle joint_trajectory_controller reports status RUNNING
Completed trajectory execution with status RUNNING
```

Evidence:

```text
/home/robross/.ros/log/move_group_1495244_1786446802666.log:44070-44086
```

The relevant log timestamps themselves move backward from approximately
`1786447339.154` to `1786447339.114`. Meanwhile, the controller accepted and
successfully completed the same goal:

```text
/home/robross/.ros/log/aubo_ros2_control_node_1495162_1786446800464.log:7771-7773
```

The painter received MoveIt's failed/RUNNING result, stopped the group, marked
contact uncertain, and performed a successful bounded retreat. This was the
correct conservative response to the result it received.

The queue-full event near this period was a single dropped ServoJ cycle and did
not cause the MoveIt timeout. ServoJ timing immediately around the failure
remained near 8 ms, all reported return codes were OK, and the controller goal
completed successfully.

### Multiple executors ruled out

The original timestamp ordering initially appeared to show overlapping command
indices. Process evidence rules that out. Bag 2's executor received SIGINT,
failed to shut down cleanly, and was forcibly killed with SIGKILL before bag 3
started:

```text
/home/robross/.ros/log/2026-08-11-19-08-28-474793-longROBROSSdd-XiaoXinPro-14-AHP9-1493247/launch.log:4-9
```

Bag 3 had one painter process. Its apparently out-of-order command timestamps
were another effect of the wall clock moving backward.

## Finding 5: Controller Success Was Not Measured-State Success

The 125 Hz trajectory-controller profile contains no per-joint trajectory or
goal tolerances:

```yaml
constraints:
  stopped_velocity_tolerance: 0.0
  goal_time: 0.0
```

Source:

```text
../aubo_ros2_driver/aubo_ros2_driver/config/aubo_controllers_125hz.yaml:36-38
```

With no per-joint goal constraints, the trajectory controller can report `Goal
reached, success!` when its time-based reference finishes even if measured
position is frozen. This is why bag 2 contains successful action results after
the driver stopped sending commands to the robot.

The painter's endpoint checks were compensating for this open-loop acceptance.
Disabling those checks exposed the controller configuration gap.

## Finding 6: The August 12 Runs Reproduced Both Failure Modes

The three August 12 recordings were made with the same installed painter and
the same 3,504-command input file:

```text
/home/robross/robross_aubo_ws/src/RobRoss/output/robot_detail.json
```

Their bag intervals were:

| Bag | Start | Duration | `/joint_states` messages |
| --- | --- | ---: | ---: |
| `8-12_Robot_1_noCheck` | 2026-08-12 14:11:07 CST | 103.143 s | 12,769 |
| `8-12_Robot_2` | 2026-08-12 14:16:26 CST | 739.502 s | 92,315 |
| `8-12_Robot_3` | 2026-08-12 14:43:42 CST | 526.586 s | 65,819 |

### Run 1: checks disabled after an RTDE fault

Run 1 started with every optional conformance check disabled:

```text
/home/robross/.ros/log/painting_executor_1732959_1786515094056.log:7
```

The driver latched the RTDE stale-state fault at Unix time
`1786515147.315`:

```text
/home/robross/.ros/log/aubo_ros2_control_node_1732473_1786515037758.log:632
```

Bag evidence after the fault:

- the joint position became exactly constant for the remaining 23.544 seconds;
- 2,943 subsequent `/joint_states` messages contained `NaN` for all six
  velocities;
- the state publisher continued near 125 Hz;
- the controller continued accepting goals and reporting success;
- the painter continued accepting one-shot endpoints because every optional
  endpoint error criterion and settling were disabled.

The one-shot diagnostics show the transition from healthy tracking to a
continuing command stream against frozen measured state. Tip error was 0.395 mm
immediately before the fault, then grew through 1.682, 3.992, 7.169, 33.917,
56.065, and finally 66.835 mm:

```text
/home/robross/.ros/log/painting_executor_1732959_1786515094056.log:1142-1232
```

The largest recorded errors were 8.451 degrees at a joint and 66.835 mm at the
tool tip. The operator interrupted while command 203 was executing. The painter
did not terminate after SIGINT or SIGTERM and was killed with SIGKILL:

```text
/home/robross/.ros/log/2026-08-12-14-11-33-841062-longROBROSSdd-XiaoXinPro-14-AHP9-1732956/launch.log:4-9
```

This is a stronger reproduction of August 11 bag 2: receipt-fresh cached
positions can allow execution to proceed after physical ServoJ streaming has
stopped when all optional endpoint checks are disabled.

### Run 2: enabled settling stopped the RTDE failure

Run 2 started with all conformance checks enabled:

```text
/home/robross/.ros/log/painting_executor_1735222_1786515393338.log:7
```

The driver latched the same RTDE fault at Unix time `1786516086.965`, during
command 2203:

```text
/home/robross/.ros/log/aubo_ros2_control_node_1734916_1786515368917.log:6984
```

The last valid position then remained unchanged through the end of the bag, and
4,869 `/joint_states` messages contained non-finite velocity. The controller
nevertheless reported `Goal reached, success!` for command 2203, reproducing the
open-loop controller-acceptance gap.

Unlike run 1, endpoint settling rejected the result. It inspected 75 samples,
found zero qualifying samples, and reported unavailable joint speed because the
velocity values were non-finite:

```text
/home/robross/.ros/log/painting_executor_1735222_1786515393338.log:13238-13250
```

The painter aborted the stroke and then refused an autonomous retreat because
it could not establish fresh, stationary robot state. That fail-closed response
was appropriate. The 25 ms dwell did not cause this timeout: no sample could
qualify at any dwell because feedback velocity had been explicitly invalidated.

Run 2 also contained three isolated one-cycle ServoJ queue-full drops, but they
were separated in time and command streaming recovered after each. They did not
cause the final fault. Source RTDE inter-arrival telemetry is still required to
identify the lower-level cause of the watchdog trip.

### Run 3: wall-clock correction caused a false MoveIt timeout

Run 3 also started with every conformance check enabled. Its archived profile
only increased endpoint settle dwell from 25 to 35 ms. The driver log contains
no RTDE watchdog fault, non-finite velocity, ServoJ exception, queue-full drop,
or abnormal ServoJ return code. All 65,819 `/joint_states` velocity messages
were finite.

The bag instead contains three backward jumps in both receipt and embedded
header timestamps:

| Approximate time | Backward jump |
| --- | ---: |
| 14:44:14 CST | 625.919 ms |
| 14:50:58 CST | 473.633 ms |
| 14:52:05 CST | 544.646 ms |

The same stream also contains compensating apparent forward gaps of 428.316,
706.909, and 475.134 ms. Controller-state timestamps show the corresponding
forward gaps. The system journal repeatedly logged:

```text
systemd-resolved: Clock change detected. Flushing caches.
```

The final backward jump occurred while command 1556 was executing. MoveIt sent
and had the goal accepted, then declared `waitForExecution timed out` while the
controller handle still reported `RUNNING`. The adjusted system timestamps put
the timeout only 87 ms after goal acceptance; that apparent interval is itself
invalid because the clock moved backward during the wait:

```text
/home/robross/.ros/log/move_group_1743736_1786516984244.log:27011-27027
```

The trajectory controller reported success approximately 38 ms after the
painter received MoveIt's failure. MoveIt's controller handle observed the
successful finish approximately 61 ms after its timeout log. ServoJ diagnostics
remained nominal around 8 ms with all return codes OK:

```text
/home/robross/.ros/log/aubo_ros2_control_node_1743655_1786516982183.log:4947-4959
```

The painter conservatively treated contact state as uncertain, rejected the
stroke, and executed a successful bounded retreat to hover:

```text
/home/robross/.ros/log/painting_executor_1744007_1786517025894.log:9356-9372
```

Endpoint settling was never reached for the failed stroke, so changing dwell
from 25 to 35 ms cannot explain this failure. Run 3 independently confirms the
August 11 bag 3 diagnosis: system-time correction can make MoveIt's action wait
expire while a healthy controller is still completing the goal.

### Cross-run conclusions

The August 12 evidence strengthens the original findings:

- the RTDE interruption recurred after the August 11 investigation and is not a
  one-off bag artifact;
- the RTDE fault is independent of whether optional conformance checks are
  enabled;
- disabling endpoint checks can allow tens of millimeters of commanded-versus-
  measured divergence after the driver has stopped physical writes;
- enabled settling detects the invalid feedback and stops the command pipeline;
- the host clock remains unstable enough to produce false MoveIt execution
  timeouts even when RTDE and ServoJ delivery are healthy;
- increasing settle dwell does not address either root cause;
- every August 12 driver instance still logged `Could not enable FIFO RT
  scheduling policy`.

## Required Fixes

### 1. Make feedback health mandatory in the painter

Optional geometric conformance toggles must not disable hardware-health
validation.

- Reject a post-execution sample containing explicitly non-finite velocity,
  even when endpoint settling is disabled.
- Keep position freshness mandatory, but do not let receipt of a republished
  faulted sample prove source freshness.
- Add an explicit driver-health signal or diagnostic subscription so the
  painter can distinguish valid stationary feedback from cached fault state.
- Abort before planning another command after any hardware-health failure.
- Keep collision, model bounds, elbow/posture, guarded-joint, and fresh-state
  checks mandatory.

Disabling settling should mean "do not wait for stationary dwell," not "accept
invalid velocity or known driver fault state."

### 2. Separate Cartesian planning failure from post-execution failure

Replace the single boolean result with a result that distinguishes at least:

- planning or Cartesian fraction failure before execution;
- trajectory validation failure before execution;
- execution failure;
- post-execution feedback or endpoint-validation failure.

Allow the pen-up joint-space fallback only for pre-execution Cartesian planning
or validation infeasibility. Never replan automatically after execution began or
after endpoint state became uncertain.

Also log the numeric `MoveItErrorCode` rather than only the generic
`Execution reported failure` message.

### 3. Configure controller measured-state tolerances

Add qualified per-joint trajectory and goal tolerances, a finite stopped
velocity tolerance, and a bounded goal-time allowance. Values must come from
stable hover evidence and should first be tested without contact.

The required behavior is:

- a frozen or materially lagging measured state causes the controller action to
  fail;
- a normal endpoint with known physical lag succeeds within the goal-time
  allowance;
- invalid velocity cannot be interpreted as a successful stopped endpoint.

This provides an independent controller-level guard instead of relying solely
on painter endpoint checks.

### 4. Instrument and qualify RTDE state delivery

Do not weaken or remove fail-closed stale-state detection. Add enough evidence
to tune it correctly:

- monotonic timestamp for every source callback;
- current and maximum source age;
- inter-arrival mean, p95, p99, and maximum;
- count of intervals over 10, 20, 50, and 100 ms;
- requested and observed RTDE rates;
- fault-time age and preceding interval history;
- callback/parser exceptions or malformed vector sizes.

The driver requests 500 Hz while the robot consistently delivers about 200 Hz.
Changing the request to 200 Hz is a justified controlled experiment because it
matches observed delivery, but it is not proven by itself to fix the stalls.

Keep the 50 ms watchdog for the first instrumented test. If otherwise healthy
runs demonstrate rare benign intervals over 50 ms, qualify a limit up to the
existing 100 ms maximum using hover-only testing. Do not raise it merely to
suppress faults.

### 5. Stabilize the hardware host

Before the next run:

- configure realtime scheduling permissions so controller manager no longer
  logs `Could not enable FIFO RT scheduling policy`;
- verify the control thread actually runs under FIFO after login/reboot;
- use the performance power profile during qualification;
- synchronize wall time before starting ROS;
- prevent wall-clock steps for the duration of robot motion;
- restore normal time synchronization after the run;
- monitor and record clock-offset changes using a monotonic reference.

Stopping time synchronization during motion is acceptable only after the clock
has synchronized and when all ROS processes are on this host. A better permanent
solution is a time service configured to slew during operation and step only at
boot or before the robot stack starts.

Do not disable MoveIt's execution-duration monitoring to hide the clock issue.
That would remove a safety mechanism instead of fixing the host clock.

### 6. Record runtime provenance

At painter and driver startup, log or save:

- executable path and ELF build ID;
- RobRoss and driver git revisions plus dirty status;
- hashes of the effective calibration and canvas YAML files;
- robot-description and SRDF hashes;
- selected robot type;
- joint names and order from each interface;
- tool offset and canvas transform;
- controller update rate and ServoJ `t`;
- requested RTDE rate and stale-state limit;
- enabled/disabled conformance checks;
- current realtime scheduling policy and clock-synchronization status.

This avoids relying on mutable files in the home directory after a run.

## Validation Sequence

1. Add unit tests proving `NaN` velocity is rejected with every optional
   conformance check disabled.
2. Add a test proving post-execution endpoint failure never invokes the
   joint-space fallback.
3. Add driver tests for RTDE inter-arrival accounting and fault reports.
4. Add controller tests showing frozen measured state cannot return success.
5. Configure realtime permissions and stable clock behavior on this machine.
6. Run the driver stationary for longer than the previous fault intervals while
   collecting source RTDE inter-arrival telemetry.
7. Run a full sketch at hover height with all endpoint and execution checks
   enabled.
8. Confirm there are no backward-clock warnings, MoveIt execution timeouts,
   RTDE faults, non-finite velocities, or unexplained state gaps.
9. Run a short single-line contact test.
10. Only after those gates pass, repeat a longer contact drawing.

## Safety Conclusions

- August 11 bag 1 and August 12 run 2 show that settling prevents a frozen,
  invalid endpoint from being accepted. It should remain enabled for hardware
  qualification.
- August 11 bag 2 and August 12 run 1 demonstrate that disabling all endpoint
  checks can allow the software pipeline to continue after physical command
  streaming has stopped. August 12 run 1 reached 66.835 mm measured tip error
  before the operator interrupted it.
- `NaN` velocity and driver fault state are hardware-health failures, not
  optional conformance metrics.
- Both clock-failure runs conservatively aborted; August 12 run 3 also completed
  a successful retreat. Host wall-clock stability must be fixed so valid
  controller executions are not spuriously aborted during contact.
- Neither changing settle dwell nor weakening MoveIt's execution monitoring
  addresses the observed clock or RTDE root causes.
- Collision checking and trajectory geometry were not the primary causes of
  these six failures and must not be weakened as part of remediation.
