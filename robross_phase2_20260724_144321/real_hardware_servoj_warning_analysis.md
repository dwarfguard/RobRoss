# Real-Hardware ServoJ Warning Analysis

**Session:** `robross_phase2_20260724_144321`  
**Observed:** 2026-07-24 14:35 local time  
**Robot:** Aubo i5 at `192.168.32.101`  
**Profile:** 200 Hz controller with ServoJ `t=0.005 s`

## Launch Command

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE robot_ip:=192.168.32.101 \
  use_fake_hardware:=false \
  controllers_file:=aubo_controllers.yaml servoj_time:=0.005
```

## Safety Decision

The driver connected and shut down cleanly, but this run failed the Phase 2
timing gate. Do not proceed to painting or hover trajectories using this result.
Stopping the driver was correct.

The new diagnostics appear to expose existing queue and scheduling behavior
rather than a robot connection failure.

## Findings

### 1. The installed description did not forward `servoj_time`

Startup reported:

```text
servoj_time hardware parameter not set; defaulting to 0.0050 s
```

The checked-out description Xacros contain the `servoj_time` argument and
hardware parameter, but the copies under the workspace `install/` directory do
not. The launch argument was therefore accepted by the launch file and then
lost while generating `robot_description`.

The fallback happened to equal the requested `0.005 s` in this run, so it did
not cause the measured 200 Hz timing problem. It would, however, make a
`125 Hz / 0.008 s` trial invalid because the driver would continue using the
`0.005 s` fallback.

The likely build-system cause is that `aubo_description/CMakeLists.txt` runs
`xacro.sh` only during CMake configuration. Pulling or incrementally building
after Xacro changes can leave generated and installed copies stale.

### 2. The 200 Hz loop ran at approximately 159 Hz

The repeated timing reports showed a mean control period near `6.28 ms`:

```text
servoj_mismatch ratio=1.25..1.28 measured_ms=6.26..6.40 t_ms=5.00
```

This is approximately `159 Hz`, or about 80 percent of the configured 200 Hz
rate. It fails the documented requirement that actual update rate reach at
least 95 percent of the configured rate.

Do not change ServoJ `t` to `0.00628 s`. The measured period is substantially
shaped by queue blocking and scheduler catch-up behavior; it is not a valid
interpolation target.

### 3. Queue-full retries dominate the timing disturbance

Each 400-cycle report contained approximately 112 to 121 queue-full events:

```text
qf_events=112..121 qf_retries=112..121
```

This affects roughly 28 to 30 percent of cycles. The driver currently handles
SDK return code `2` by sleeping for 5 ms and retrying the same command in an
unbounded loop. That sleep occurs in the ros2_control write thread and directly
creates late control cycles, commonly near 12 ms.

The `rc=ok:400` field does not mean no queue-full response occurred. It records
the final successful return after transient queue-full retries; the `qf_*`
fields preserve those intermediate responses.

### 4. Deadline catch-up can reinforce queue saturation

The control loop advances an absolute deadline every iteration. After a write
blocks on queue-full, the next iterations can run at approximately 3 ms while
the scheduler catches up. These bursts may refill the robot queue and create a
feedback loop of long blocked cycles followed by short catch-up cycles.

### 5. RTDE is requested at 500 Hz but arrives near 200 Hz

The driver requests the robot-state RTDE recipe at 500 Hz. The SDK reports
approximately 1,000 packets in five seconds rather than the requested 2,500:

```text
The chanel 99 of RTDE received 1000 packages less than 2500 in 5 seconds
```

This indicates a continuously active stream near 200 Hz, not an RTDE
disconnect. Requesting the observed supported rate of 200 Hz should remove the
warning noise while retaining the available feedback rate.

### 6. FIFO scheduling was unavailable

```text
Could not enable FIFO RT scheduling policy
```

The missing real-time scheduling privilege can contribute jitter and should be
fixed before final acceptance testing. It does not by itself explain the close
correlation between queue-full and late-cycle counts.

## Warning Classification

### Actionable

- Missing `servoj_time` hardware parameter due to stale installed Xacros.
- ServoJ mean period around `6.28 ms` against a `5.00 ms` target.
- Queue-full responses in roughly 28 to 30 percent of cycles.
- Control-loop catch-up bursts after blocked writes.
- RTDE state recipe requested above the observed delivery rate.
- FIFO real-time scheduling unavailable.

### Non-Fatal Or Expected In This Run

- Direct `robot_description` parameter deprecation.
- `allow_nonzero_velocity_at_trajectory_end` deprecation.
- RTDE `Operation aborted` after Ctrl-C during normal shutdown.
- The misspelling `chanel`, which originates in the Aubo SDK.

## Recommended Recovery And Verification

### 1. Force a clean CMake reconfiguration

From the workspace root:

```bash
colcon build --packages-select aubo_description aubo_ros2_driver \
  --cmake-clean-cache
source install/setup.bash
```

Confirm that the installed `aubo_ros2.xacro` and
`aubo.ros2_control.xacro` now contain `servoj_time`.

### 2. Verify the argument path without connecting to the robot

Run the malformed-value loopback checks from the hardware first-run guide. A
`nan` value must fail with `is not a finite number`, and `0.005junk` must fail
with `has trailing characters after the number`. If either launch instead logs
the default fallback, the installed description is still stale.

### 3. Run the 125 Hz / 8 ms profile above the paper first

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=$AUBO_TYPE robot_ip:=192.168.32.101 \
  use_fake_hardware:=false \
  controllers_file:=aubo_controllers_125hz.yaml servoj_time:=0.008
```

Startup must report:

```text
servoj_config t=0.0080
```

It must not report that the hardware parameter is missing. Stop immediately if
any queue-full event, nonzero final ServoJ return code, timing fault, or visible
wrist oscillation occurs.

### 4. Harden the driver and build path

- Regenerate description artifacts whenever a source URDF or Xacro changes.
- Add an installed-Xacro integration test that expands `servoj_time:=0.008` and
  verifies the resulting hardware parameter.
- Treat a missing `servoj_time` as a physical-hardware initialization error
  instead of silently falling back.
- Derive or validate controller update rate and ServoJ `t` from one
  authoritative configuration.
- Replace unbounded stale-command retry with bounded handling that preserves
  the newest command and faults on persistent saturation.
- Resynchronize the control-loop deadline after an overrun instead of issuing
  burst catch-up writes.
- Record exact per-attempt ServoJ timestamps, return codes, and queue-only delay.
- Request the verified RTDE feedback rate, initially 200 Hz rather than 500 Hz.
- Configure and verify FIFO scheduling before final comparisons.

## Acceptance Gates

Before proceeding beyond above-paper testing, require all of the following:

- Startup reports the requested ServoJ `t`, with no fallback message.
- Actual update rate is at least 95 percent of the configured rate.
- No queue-full event occurs.
- No unexplained ServoJ return code or exception occurs.
- No timing fault or RTDE under-rate warning occurs.
- No visible movement-synchronized wrist oscillation occurs.
- Tracking and canvas-normal limits in the Phase 2 hardware guide pass.
