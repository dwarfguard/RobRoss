# Hardware Run Issue Summary: 2026-08-11 Bags

This report summarizes the three most recent hardware recordings:

- `8-11_Bag`
- `8-11_Bag_2_no_checks`
- `8-11_Bag_3_no_checks`

The two `no_checks` runs disabled Cartesian deviation, Cartesian normal
deviation, Cartesian orientation deviation, execution joint error, execution
tip error, execution tip orientation error, and endpoint settling. Collision,
bounds, jump, posture, and guarded-joint checks remained enabled.

## Summary

| Bag | Result | Last recorded command | Main issue |
| --- | --- | --- | --- |
| `8-11_Bag` | Executor aborted | `2957/3249`, `move_to`, `sketch_detail_189` | Endpoint settling failed twice; the measured endpoint did not become valid and stationary. |
| `8-11_Bag_2_no_checks` | No executor failure recorded; bag stopped | `867/3249`, `paint_stroke`, `sketch_outline_66` | Logged endpoint error grew to approximately 36 mm, but the drawing itself appeared relatively precise. This makes a real TCP error uncertain and suggests a diagnostic/runtime-state issue. |
| `8-11_Bag_3_no_checks` | Executor aborted | `2496/3249`, `paint_stroke`, `sketch_detail_114` | Cartesian execution failure coincided with an approximately 653 ms pause in both feedback topics. |

All three runs had nominal ServoJ timing near 125 Hz and no non-OK ServoJ
return codes. ServoJ timing alone does not explain the failures.

## `8-11_Bag`

The enabled endpoint-settling check rejected command 2957:

```text
Endpoint failed to settle (Cartesian): 187 samples, 0 qualifying,
best error ratio 3.84, last joint speed unavailable (timed out)
Endpoint failed to settle (Joint-space): 187 samples, 0 qualifying,
best error ratio 3.87, last joint speed unavailable (timed out)
Command 2957 ... failed, aborting
```

The same measured endpoint was reported repeatedly, and no fresh valid
stationary sample was obtained before timeout. The failure is therefore most
consistent with the arm reaching or reporting an endpoint that did not match
the requested trajectory endpoint, followed by a settling timeout.

ServoJ evidence was generally healthy: 184,249 successful cycles, no bad
return codes, and one queue-full event. There is no evidence that a ServoJ
return-code failure directly caused this abort.

## `8-11_Bag_2_no_checks`

The bag contains no executor failure or abort message. It ends while command
867 is being executed. The executor/controller repeatedly reported successful
goals, while the one-shot endpoint diagnostic reported increasing error:

| Command | Reported joint error | Reported tip error |
| ---: | ---: | ---: |
| 856 | 2.706 deg | 28.762 mm |
| 860 | 2.985 deg | 32.088 mm |
| 864 | 3.026 deg | 35.243 mm |
| 867 | 2.944 deg | 36.049 mm |

The recorded drawing was reported to be relatively precise and did not show
an obvious progressively displaced path. Consequently, the logged 30--36 mm
tip error should not yet be treated as a confirmed physical TCP error.

Likely explanations include:

- a stale or different `painting_executor` executable than the current source;
- an error calculated against the wrong final trajectory waypoint;
- stale or incorrect internal planning state;
- joint-name/order association problems in the runtime diagnostic;
- a mismatch between the runtime URDF/tool offset and the model used for the
  diagnostic.

ServoJ and feedback rates were nominal in this run: all 27,000 ServoJ return
codes were OK, with no queue-full events or diagnostic warnings. That makes a
transport failure less likely.

The next verification should independently reconstruct the TCP from
`/joint_states`, the bag's `/robot_description`, and the runtime
`tool_offset_xyz`. Compare that result with `/robross_markers`, controller
feedback, controller reference, and the executor's endpoint log. Also record
the executable path, build revision, joint names, URDF, and tool offset at
startup so stale-runtime issues can be ruled out.

## `8-11_Bag_3_no_checks`

The executor aborted during command 2496:

```text
Execution reported failure (Cartesian); contact state is uncertain
Stroke rejected; refusing an automatic posture change while painting
Command 2496 ... failed, aborting
```

Immediately before the failure, command 2495 had small reconstructed tracking
errors, so the run does not look like a gradual geometric tracking failure.

The strongest evidence is a simultaneous feedback interruption:

- `/joint_states` gap: approximately 652.57 ms;
- `/joint_trajectory_controller/controller_state` gap: approximately 652.64 ms;
- normal topic intervals: approximately 8--16 ms.

ServoJ recorded one queue-full/drop event near the same period, although all
reported ServoJ return codes remained OK. Possible causes include:

- rosbag2/SQLite recorder write stall;
- ROS/DDS scheduling or executor stall;
- controller process stall;
- hardware-interface or robot-network interruption;
- system CPU or I/O pressure.

The bag alone cannot distinguish a publisher pause from a recorder pause.

To distinguish them, compare each message's bag timestamp with its embedded
`header.stamp` around Unix time `1786447138.333`. If header timestamps advance
normally while bag timestamps jump, the recorder/storage path is the leading
suspect. If both jump, investigate the controller, hardware interface, DDS,
or system scheduling.

During a reproduction, collect controller and hardware-interface logs outside
the bag and monitor:

```bash
top
iostat -xz 1
vmstat 1
pidstat -dur 1
```

Also check system logs for the corresponding wall-clock interval and inspect
ServoJ queue-full timing relative to the feedback gap.

## Recommended follow-up order

1. Verify the exact `painting_executor` binary and build revision used for
   `8-11_Bag_2_no_checks`.
2. Recompute TCP motion independently from `/joint_states` and compare it to
   markers, controller reference, and feedback.
3. Extract bag timestamps and message header timestamps around the 653 ms gap
   in `8-11_Bag_3_no_checks`.
4. Reproduce run 3 while recording controller, hardware-interface, system,
   and rosbag2 logs separately.
5. Re-enable endpoint and execution checks for subsequent hardware runs; the
   run-1 evidence shows endpoint settling can prevent an invalid state from
   being accepted.

