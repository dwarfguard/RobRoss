# KWR75D Force-Controlled Teaching and Marker Painting

## Summary

Use Aubo's controller-native force/admittance loop for contact control while ROS
continues supplying planned tangential motion. The first milestone will provide:

- guarded force touch for canvas teaching;
- constant canvas-normal force while drawing with the hard-tip acrylic marker;
- standard ROS wrench telemetry and fail-closed supervision;
- six-axis force profiles that can later support angled, compliant brush
  strokes;
- an action interface a future camera node can use to refine vision-derived
  canvas points in Z.

Do not implement a workstation-side force PID. Aubo already supplies force-mode,
target-wrench, dynamics, filtering, and controller-side supervision APIs. Force
mode also disables some controller protections, including collision detection,
so existing MoveIt validation plus force/position/orientation/speed supervision
must remain mandatory. See the
[Aubo force-control reference](https://docs.aubo-robotics.cn/arcs_api/en/force__control_8h.html).

## Implementation Changes

### 1. Qualify and expose the KWR75D

- Add a read-only qualification utility using the bundled Aubo SDK:
  - verify the controller exposes `kw_ftsensor`;
  - record controller/ARCS and SDK versions;
  - read sensor mounting pose, payload/CoM, force offset, force-mode state, and
    live six-axis samples;
  - require explicit `--apply-calibration` before changing controller
    configuration.
- Treat calibration as untrusted initially. Perform Aubo's multi-orientation
  sensor calibration with the complete distal load—sensor, claw, holder, and
  marker—then save the resulting sensor pose, payload/CoM, offset, robot
  identity, and date in a hardware-specific YAML.
- Re-run pen-tip TCP calibration after the sensor/tool stack is finalized, then
  re-teach the canvas.
- Extend the Aubo RTDE subscription with `R1_actual_TCP_force_sensor`; reject
  missing, non-six-element, non-finite, or stale samples.
- Export six ros2_control sensor interfaces and run the standard force-torque
  broadcaster, publishing:
  - `/kwr75d/wrench` as `geometry_msgs/WrenchStamped`;
  - frame `kwr75d_link`, defined from the same sensor mounting pose used by the
    controller.
- Verify the RTDE field's bias and coordinate semantics experimentally before
  declaring the frame valid: unloaded stability, known pushes on
  ±X/±Y/±Z, and several tool orientations.

### 2. Add a controller-native force manager

Implement an `aubo_force_control_node` in the Aubo driver package, backed by a
mockable SDK adapter. It owns all `setDynamicModel`, `setTargetForce`, filtering,
supervision, `fcEnable`, and `fcDisable` calls; other nodes must not call these
APIs directly.

Provide:

- `GuardedContact.action`
  - Goal: profile name, force-frame pose, maximum travel, timeout, and whether
    to remain in force hold.
  - The +Z axis of the supplied frame is the approach direction.
  - Result: success/error code, measured contact pose, settled wrench, and
    travel.
  - Feedback: phase, wrench, travel, and sensor age.
- `SetForceMode.srv`
  - Inputs: profile name, force-frame pose, enable/disable, and
    emergency-disable flag.
  - Normal disable is accepted only after the release-force condition is met;
    emergency disable acts immediately.
- Force profiles in YAML containing required six-axis compliance, target
  wrench, M/D/K, speed limits, low-pass filter, force limits, position box,
  orientation limit, TCP-speed limit, settle band/time, stale timeout, and
  maximum enabled duration.
- No usable real-contact defaults. Hardware force profiles remain disabled
  until qualification writes explicit values.
- Controller-side timeout and travel/force supervision on every force-mode
  activation so loss of the ROS process cannot permit unbounded motion.
- Target-force ramps for enable, profile changes, and release; never step
  directly between wrench targets.

The bundled SDK 0.24.x API subset—rather than newer convenience APIs such as
`fcContact`—is the compatibility target: `setTargetForce`, `setDynamicModel`,
`setSupvForce`, `setSupvPosBox`, `setSupvOrient`, `setSupvTcpSpeed`, `fcEnable`,
and `fcDisable`. The official API confirms these facilities and controller-side
termination/supervision support. See the
[Aubo ForceControl module](https://docs.aubo-robotics.cn/arcs_api/en/group__ForceControl.html).

### 3. Force-assisted canvas teaching

- Retain freedrive for coarse placement.
- Replace final manual nudges with `GuardedContact` using a low-force
  `marker_teach` profile aligned to the current tool +Z axis.
- Accept contact only when:
  - the wrench stream is fresh;
  - normal force enters the settle band for the configured duration;
  - tangential forces and torques remain below limits;
  - travel and timeout limits are not exceeded.
- Record the action's settled contact pose directly in `teach_canvas.py`; then
  ramp target force to zero, retreat, and disable force mode before freedrive
  resumes.
- Use `plane_bias_mm: 0` for force-enabled calibration. The old 1 mm positional
  preload remains available only for legacy position-controlled painting;
  force mode supplies preload from the target wrench.
- Preserve four-corner and interior-sample plane fitting and all existing
  flatness/skew checks.
- Keep the future camera boundary simple: vision may later provide approximate
  corner hover poses and X/Y orientation; `GuardedContact` resolves each
  point's physical Z and returns the contact pose.

### 4. Constant-force marker painting

- Add an optional `force_profile` to `lower_tool`; default it from the hardware
  profile. The selected profile remains active until `lift_tool`.
- For the marker milestone:
  - canvas X/Y and tool orientation remain position-controlled;
  - only canvas-normal Z is compliant;
  - use the taught canvas frame as `FRAME_FORCE`, whose +Z already points into
    the paper;
  - continuously log target and measured normal force.
- Execute each contact segment with this state machine:
  1. move to the existing safe hover with force mode off;
  2. guarded-contact using the selected marker profile;
  3. execute the validated tangential MoveIt path while native force hold
     remains active;
  4. ramp target force to zero;
  5. retreat along the measured canvas normal while force mode remains in the
     release profile;
  6. disable force mode only after clearance and low force are confirmed.
- Abort the trajectory and enter the existing measured-retreat path on stale
  wrench data, overforce, persistent loss of contact, unexpected force-mode
  state, SDK failure, orientation violation, or MoveIt/controller failure.
- Preserve the profile as a full six-axis definition. Future brush profiles
  may enable rotational compliance and different wrench/orientation settings
  without changing the force manager; variable force within a stroke remains a
  later path-format extension.

## Tests and Commissioning

- Unit tests:
  - RTDE force parsing, sensor interface export, NaN/length/staleness rejection;
  - frame/sign transforms and canvas-normal force projection;
  - profile validation and target-force ramping;
  - guarded-contact success, timeout, overtravel, overforce, and SDK-error
    paths;
  - idempotent and emergency force disable;
  - teaching records the settled action pose and retains plane-fit validation;
  - painting enforces hover → contact → hold → release → retreat → disabled
    ordering.
- Integration tests with a mocked SDK and fake hardware:
  - force mode cannot start without a fresh calibrated sensor;
  - loss of any ROS participant produces a bounded stop;
  - `lower_tool`/`paint_path`/`lift_tool` cannot execute out of order;
  - optional force profiles remain backward-compatible with existing path
    JSON.
- Hardware gates:
  1. no-motion sensor logging and calibration verification;
  2. zero-target force mode while stationary;
  3. hover-only tangential ServoJ trajectory with force mode enabled;
  4. guarded touch against a compliant test surface;
  5. sacrificial-paper stationary force hold;
  6. short straight marker line, then curves/corners, then complete artwork.
- The hover test must prove controller-native force mode coexists with the
  current ServoJ/JTC path without violating the repository's existing timing,
  queue-full, or endpoint gates. If not, block contact work; do not substitute
  a user-space PID or weaken those gates.
- Determine marker touch force, drawing force, settle tolerance, and hard
  overforce limit from measured noise and sacrificial tests, then commit them
  only to the hardware-specific profile. Acceptance requires repeatable
  contact depth, visually consistent line weight, bounded force error, no
  supervision event, and complete force/trajectory logs.

## Assumptions

- Initial tool is a hard-tip acrylic marker; only normal translational
  compliance is enabled.
- Camera integration is outside this milestone, but the guarded-contact action
  is its supported Z-refinement interface.
- Existing MoveIt collision, joint-family, path-deviation, and endpoint checks
  remain unchanged.
- Contact remains prohibited until both the existing Phase 2 motion gates and
  the new force-control commissioning gates pass.
- Sensor selection or calibration is never silently overwritten at startup.
