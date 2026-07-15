# robross_painter Configuration Reference

Robot-specific settings are ROS parameters loaded from a calibration profile.
Artwork profiles under `configs/` do not contain robot geometry or motion
limits.

## Canvas And Tool

| Parameter | Meaning |
| --- | --- |
| `canvas_origin_xyz` | Paper top-left corner on the contact plane, in `base_link` meters. |
| `canvas_quat_xyzw` | Full canvas orientation: x right, y down, z into the paper. A taught value overrides `canvas_x_yaw_deg`. |
| `canvas_x_yaw_deg` | Flat-paper fallback for the page x-axis direction in the base XY plane. |
| `safe_clearance_m` | Pen-up travel distance from the paper along the canvas normal. |
| `tool_offset_xyz`, `tool_offset_rpy` | Pen-tip pose in `ee_link`. Zero treats `ee_link` as the tip. |
| `tool_spin_deg` | Claw rotation about the pen axis. |

## Collision Scene

| Parameter | Meaning |
| --- | --- |
| `ground_enabled` | Adds a large ground collision plane at `ground_z_m`. Disable when the taught canvas lies on or near the mounting surface and the backing patch protects it instead. |
| `ground_z_m` | Top of the ground collision plane in `base_link`. |
| `canvas_backing_enabled` | Adds a collision plane behind the paper. Enable it for a wall or board. |
| `canvas_backing_clearance_m` | Gap between the drawing plane and backing collision surface. |
| `canvas_backing_size_xy_m` | Backing patch size in the canvas plane. `[0, 0]` auto-sizes to the canvas plus `canvas_backing_margin_m` per side. |
| `canvas_backing_margin_m` | Margin added around the canvas when the backing size is auto. |
| `claw_collision_size_xyz` | Size of the claw's attached collision box. `[0, 0, 0]` disables it. |
| `claw_collision_offset_xyz` | Claw-box center in `ee_link`. |
| `claw_touch_links` | Robot links allowed to touch the attached claw object. Usually left at the executor defaults. |

The executor validates claw dimensions and refuses startup if the configured box
would intersect the backing at pen contact. At startup it also reconciles the
three objects it owns (`ground_plane`, `canvas_backing`, and `pen_claw`): an
object disabled by the active profile is removed from MoveIt's planning scene.

## Planning And Execution

| Parameter | Meaning |
| --- | --- |
| `velocity_scaling`, `acceleration_scaling` | MoveIt trajectory scaling factors. Use `0.1` for first contact. |
| `eef_step_m` | Cartesian interpolation resolution. |
| `cartesian_jump_threshold` | Relative joint-jump detector for Cartesian paths. Never set it to zero. |
| `dry_run` | Plans a coherent command sequence without sending trajectory goals. |
| `joint_states_topic` | Joint feedback topic, including any robot namespace. |
| `state_validity_service` | MoveIt state-validity service, including any namespace. |

During dry-run, each planned endpoint seeds the next command. During live
execution, measured feedback is refreshed before every motion.

## Posture And Joint Guards

| Parameter | Meaning |
| --- | --- |
| `elbow_up_enabled` | Enables the hard elbow-family invariant. |
| `elbow_joint` | Joint used to identify the elbow family. |
| `elbow_up_min_deg`, `elbow_up_max_deg` | Allowed elbow interval at startup and throughout trajectories. |
| `guarded_joints` | Joints whose displacement and travel are bounded; defaults to `shoulder_joint` and `wrist3_joint`. |
| `max_guarded_joint_goal_delta_deg` | Maximum start-to-end displacement for each guarded joint. |
| `max_guarded_joint_travel_deg` | Maximum accumulated guarded-joint travel for pen-up trajectories. |
| `max_guarded_joint_paint_travel_deg` | Tighter accumulated travel limit for lowering, painting, and lifting. |
| `max_guarded_joint_step_deg` | Maximum change between adjacent trajectory samples. |

The shipped profiles use a 120-degree goal limit, 150-degree pen-up travel
limit, 90-degree contact-motion travel limit, and 45-degree sample-step limit.
These guards supplement the Cartesian jump detector; they do not replace it.

## Trajectory Verification

| Parameter | Meaning |
| --- | --- |
| `max_cartesian_deviation_mm` | Maximum post-retiming pen-tip position error from the requested path. |
| `max_cartesian_orientation_deviation_deg` | Maximum post-retiming tool-orientation error. |
| `max_execution_tip_error_mm` | Maximum measured endpoint position error after execution. |
| `max_execution_tip_orientation_error_deg` | Maximum measured endpoint orientation error after execution. |
| `totg_path_tolerance` | Time-Optimal Trajectory Generation path tolerance. |
| `totg_resample_dt` | Retimed trajectory output interval in seconds. |

Validation runs after TOTG and before execution. It interpolates between
trajectory knots and checks pen-tip FK, model bounds, elbow posture, guarded
joints, and MoveIt collision validity. After execution, fresh measured feedback
must also satisfy model, collision, posture, and endpoint checks.

## Profiles

| File | Intended environment | Executes by default |
| --- | --- | --- |
| `config/rviz_wall_a4.yaml` | Fake hardware, virtual wall | Yes |
| `config/rviz_taught_a4.yaml` | Fake hardware, taught canvas on any plane (sim-only: no ground plane, relaxed guards) | Yes |
| `config/demo_v1_rviz.yaml` | Fake hardware, horizontal paper | Yes |
| `config/hardware_a4.yaml` | Real-arm template, any taught surface | No (`dry_run: true`) |

Treat shipped values as reviewed starting points, not proof that physical
geometry has been measured. Follow [PREFLIGHT.md](PREFLIGHT.md) before every
real-arm session.
