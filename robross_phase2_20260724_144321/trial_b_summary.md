# Tracking bag analysis

/joint_trajectory_controller/controller_state: 17069 msgs, 68.0 Hz mean (67.4 Hz median), interval 1.3/14.7/49.7 ms min/mean/max, p95 20.0 ms
/joint_states: 46128 msgs, 183.4 Hz mean (203.9 Hz median), interval 0.0/5.5/47.0 ms min/mean/max, p95 11.8 ms

| # | type | label | dir | n | speed mm/s | normal err mean/min/max mm | tang max mm | compression mean/min/max mm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 3/18 | move_to | smooth_s_curve | mixed | 637 | 25.2 | -0.393/-2.106/+0.077 | 60.945 | -20.87/-21.55/-19.00 |
| 3/18 | move_to | smooth_s_curve | static | 214 | 43.5 | -0.960/-2.106/+0.000 | 60.974 | -21.03/-21.03/-21.03 |
| 3/18 | move_to | smooth_s_curve | static | 217 | 43.3 | -0.920/-2.017/+0.000 | 61.047 | -21.03/-21.03/-21.03 |
| 3/18 | move_to | plus_y_retract_same_segment | mixed | 97 | 88.4 | -1.100/-2.028/+0.000 | 61.047 | -21.03/-21.03/-21.03 |
| 3/10 | move_to | plus_y_then_minus_y | +Y | 267 | 39.9 | -0.904/-2.028/-0.000 | 87.353 | -21.03/-21.03/-21.03 |
| 3/6 | move_to | plus_x_alternating_y_curve | +Y | 473 | 38.1 | -0.321/-2.015/+0.319 | 150.986 | -21.03/-21.03/-21.03 |

Per-joint max |error| (deg):
  [3] smooth_s_curve: foreArm_joint: 11.60, upperArm_joint: 6.23, wrist1_joint: 5.40
  [3] smooth_s_curve: foreArm_joint: 11.60, upperArm_joint: 6.23, wrist1_joint: 5.40
  [3] smooth_s_curve: foreArm_joint: 11.61, upperArm_joint: 6.23, wrist1_joint: 5.40
  [3] plus_y_retract_same_segment: foreArm_joint: 11.61, upperArm_joint: 6.23, wrist1_joint: 5.40
  [3] plus_y_then_minus_y: foreArm_joint: 14.83, upperArm_joint: 7.67, wrist1_joint: 7.15
  [3] plus_x_alternating_y_curve: foreArm_joint: 27.28, upperArm_joint: 14.58, wrist1_joint: 12.67

## Phase delay & normal oscillation
Instantaneous-direction normal error mean/min/max mm (n):
  [3] smooth_s_curve: -Y +0.001/+0.001/+0.001 (n=1); mixed -0.393/-2.105/+0.077 (n=625)
  [3] smooth_s_curve: mixed -0.894/-2.106/+0.000 (n=202)
  [3] smooth_s_curve: mixed -0.857/-2.017/+0.000 (n=203)
  [3] plus_y_retract_same_segment: +X -1.010/-2.028/-0.000 (n=86); mixed -0.976/-1.951/+0.000 (n=2)
  [3] plus_y_then_minus_y: -X -1.014/-2.028/-0.000 (n=2); mixed -0.846/-1.984/-0.000 (n=252)
  [3] plus_x_alternating_y_curve: +Y -0.281/-2.014/+0.319 (n=461); mixed -0.992/-1.985/+0.000 (n=2)

Per-segment delay and canvas-normal oscillation:
  [3] smooth_s_curve: delay n/a; normal pp seg 2.18 mm, per-cycle mean/max 2.18/2.18 mm (0 cyc), rms 0.670 mm
  [3] smooth_s_curve: delay n/a; normal pp seg 2.11 mm, per-cycle mean/max 2.11/2.11 mm (1 cyc), rms 1.157 mm
  [3] smooth_s_curve: delay n/a; normal pp seg 2.02 mm, per-cycle mean/max 2.02/2.02 mm (1 cyc), rms 1.114 mm
  [3] plus_y_retract_same_segment: delay n/a; normal pp seg 2.03 mm, per-cycle mean/max 2.03/2.03 mm (1 cyc), rms 1.286 mm
  [3] plus_y_then_minus_y: delay n/a; normal pp seg 2.03 mm, per-cycle mean/max 2.03/2.03 mm (1 cyc), rms 1.103 mm
  [3] plus_x_alternating_y_curve: delay n/a; normal pp seg 2.33 mm, per-cycle mean/max 2.33/2.33 mm (1 cyc), rms 0.782 mm
Phase 2B tracking gate: FAIL (delay: MISSING (mandatory; need an oscillatory/curved path), |normal| <=0.25 mm: FAIL)
  delay median/p95 n/a; worst |normal| 2.11 mm; worst per-cycle pp 2.33 mm (the per-cycle pp is the objective proxy for the operator's 'visible wrist oscillation' check).

## ServoJ timing (aubo_servoj_diag)
config: t=0.005 s, gain=200, window=400 cycles
60 reports over 24000 cycles; 158.0 Hz effective loop (79.0% of 200 Hz configured)
period ms: mean 6.33, min 2.65, max 19.22, p95 13.11 / p99 14.80 (worst window)
servoJoint RPC ms: mean 2.29, max 5.91; whole Servoj ms: mean 6.27, max 16.51
late cycles: 7509 (worst run 15); queue-full: 7457 events, 7457 retries, 85886.9 ms blocked
return codes: ok 24000, busy 0, bad 0, inval 0, ign 0, other 0; exceptions 0
log warnings: mismatch 76, rc 0, queue-full 0; fault latched: YES
Phase 2B timing gate: FAIL (rate >= 95%: FAIL, no queue-full: FAIL, no non-OK rc/exc: ok, no timing fault: FAIL)
  (joint-delay gate <30 ms median / <50 ms p95 is assessed separately from controller_state cross-correlation.)
