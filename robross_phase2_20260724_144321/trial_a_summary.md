# Tracking bag analysis

/joint_trajectory_controller/controller_state: 37100 msgs, 62.4 Hz mean (62.5 Hz median), interval 2.9/16.0/542.9 ms min/mean/max, p95 16.2 ms
/joint_states: 74314 msgs, 125.0 Hz mean (125.0 Hz median), interval 3.1/8.0/534.8 ms min/mean/max, p95 8.2 ms

| # | type | label | dir | n | speed mm/s | normal err mean/min/max mm | tang max mm | compression mean/min/max mm |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 3/18 | move_to | smooth_s_curve | mixed | 671 | 18.0 | -2.394/-3.312/-0.002 | 2.562 | -195.19/-330.29/-19.36 |
| 4/18 | lower_tool | smooth_s_curve | static | 43 | 0.3 | -2.542/-3.653/+0.209 | 0.129 | -11.77/-19.18/0.46 |
| 5/18 | paint_path | smooth_s_curve | +X | 268 | 41.3 | +0.014/-0.426/+0.534 | 4.314 | 1.01/0.57/1.20 |
| 6/18 | lift_tool | smooth_s_curve | static | 43 | 0.7 | +2.541/-0.035/+3.594 | 0.284 | -6.32/-18.30/1.04 |
| 7/18 | move_to | closed_circle | mixed | 265 | 30.9 | +0.105/-0.703/+0.356 | 3.574 | -18.56/-19.00/-18.11 |
| 8/18 | lower_tool | closed_circle | static | 49 | 0.5 | -2.238/-3.184/+0.008 | 0.154 | -11.07/-19.01/0.83 |
| 9/18 | paint_path | closed_circle | static | 435 | 25.2 | -0.004/-0.231/+0.200 | 3.903 | 0.99/0.77/1.19 |
| 10/18 | lift_tool | closed_circle | static | 48 | 1.2 | +2.219/-0.056/+3.014 | 0.398 | -6.79/-18.63/1.02 |
| 11/18 | move_to | sine_squiggle | +X | 39 | 28.0 | +0.009/-0.262/+0.162 | 3.531 | -18.88/-19.01/-18.69 |
| 12/18 | lower_tool | sine_squiggle | static | 50 | 0.1 | -2.204/-3.187/-0.003 | 0.131 | -11.27/-19.02/0.81 |
| 13/18 | paint_path | sine_squiggle | +X | 838 | 19.3 | -0.006/-0.331/+0.329 | 2.300 | 0.99/0.67/1.33 |
| 14/18 | lift_tool | sine_squiggle | static | 49 | 0.2 | +2.209/-0.004/+3.085 | 0.118 | -6.71/-18.68/1.01 |
| 15/18 | move_to | sharp_corners | mixed | 342 | 33.1 | +0.029/-0.207/+0.148 | 3.388 | -18.88/-19.01/-18.73 |
| 16/18 | lower_tool | sharp_corners | static | 53 | 0.2 | -2.062/-2.838/+0.003 | 0.093 | -11.09/-19.01/0.72 |
| 17/18 | paint_path | sharp_corners | +X | 750 | 24.9 | +0.011/-0.235/+0.279 | 3.389 | 1.00/0.76/1.24 |
| 18/18 | lift_tool | sharp_corners | static | 55 | 0.1 | +1.957/-0.002/+2.597 | 0.118 | -7.04/-18.73/1.01 |
| 3/18 | move_to | plus_y_retract_same_segment | mixed | 287 | 29.6 | -0.142/-0.237/+0.009 | 2.828 | -19.48/-19.72/-19.00 |
| 4/18 | lower_tool | plus_y_retract_same_segment | static | 46 | 0.8 | -2.299/-3.300/+0.064 | 0.292 | -11.31/-19.01/0.63 |
| 5/18 | paint_path | plus_y_retract_same_segment | +Y | 108 | 17.6 | +0.132/-0.190/+0.337 | 1.635 | 0.95/0.81/1.11 |
| 6/18 | lift_tool | plus_y_retract_same_segment | static | 49 | 0.8 | +2.158/-0.043/+3.009 | 0.272 | -6.74/-18.75/1.03 |
| 7/18 | move_to | minus_y_extend_same_segment | static | 6 | 1.2 | -0.102/-0.151/+0.110 | 0.091 | -18.94/-18.96/-18.89 |
| 8/18 | lower_tool | minus_y_extend_same_segment | static | 47 | 0.1 | -2.268/-3.095/-0.013 | 0.102 | -10.79/-18.95/0.77 |
| 9/18 | paint_path | minus_y_extend_same_segment | -Y | 112 | 16.9 | -0.105/-0.172/+0.126 | 1.646 | 0.83/0.74/0.99 |
| 10/18 | lift_tool | minus_y_extend_same_segment | static | 46 | 0.5 | +2.345/-0.049/+3.332 | 0.212 | -6.69/-18.61/1.04 |
| 11/18 | move_to | plus_x_control_same_segment | +Y | 188 | 19.1 | +0.093/-0.379/+0.170 | 1.904 | -18.73/-18.99/-18.47 |
| 12/18 | lower_tool | plus_x_control_same_segment | static | 48 | 1.0 | -2.243/-2.971/+0.036 | 0.329 | -11.42/-19.10/0.17 |
| 13/18 | paint_path | plus_x_control_same_segment | +X | 61 | 31.3 | +0.032/-0.367/+0.557 | 3.348 | 0.78/0.57/0.99 |
| 14/18 | lift_tool | plus_x_control_same_segment | static | 52 | 0.2 | +2.036/-0.016/+2.876 | 0.110 | -7.32/-18.93/0.99 |
| 15/18 | move_to | minus_x_control_same_segment | static | 5 | 0.0 | -0.032/-0.035/-0.023 | 0.013 | -18.96/-18.96/-18.95 |
| 16/18 | lower_tool | minus_x_control_same_segment | static | 53 | 0.1 | -2.038/-2.999/-0.001 | 0.141 | -10.82/-18.97/0.95 |
| 17/18 | paint_path | minus_x_control_same_segment | -X | 61 | 31.3 | -0.001/-0.037/+0.044 | 3.373 | 0.97/0.95/1.00 |
| 18/18 | lift_tool | minus_x_control_same_segment | static | 49 | 0.4 | +2.186/+0.001/+2.955 | 0.137 | -6.42/-18.26/1.01 |
| 3/10 | move_to | plus_y_then_minus_y | mixed | 93 | 21.5 | +0.131/-0.000/+0.204 | 2.168 | -18.92/-19.01/-18.82 |
| 4/10 | lower_tool | plus_y_then_minus_y | static | 52 | 0.7 | -2.046/-2.794/+0.012 | 0.234 | -11.36/-19.13/0.38 |
| 5/10 | paint_path | plus_y_then_minus_y | static | 195 | 18.7 | +0.002/-0.253/+0.419 | 1.806 | 0.90/0.75/1.07 |
| 6/10 | lift_tool | plus_y_then_minus_y | static | 52 | 0.6 | +2.060/-0.011/+2.726 | 0.257 | -6.85/-18.52/1.07 |
| 7/10 | move_to | minus_y_then_plus_y | mixed | 105 | 19.0 | +0.088/-0.475/+0.183 | 1.829 | -18.69/-18.99/-18.39 |
| 8/10 | lower_tool | minus_y_then_plus_y | static | 55 | 0.5 | -1.926/-2.568/+0.010 | 0.188 | -11.03/-19.05/0.59 |
| 9/10 | paint_path | minus_y_then_plus_y | static | 195 | 18.8 | +0.012/-0.202/+0.337 | 1.914 | 0.93/0.55/1.26 |
| 10/10 | lift_tool | minus_y_then_plus_y | static | 56 | 0.7 | +1.889/-0.032/+2.469 | 0.259 | -6.82/-18.53/1.02 |
| 3/6 | move_to | plus_x_alternating_y_curve | mixed | 111 | 30.8 | +0.074/-0.028/+0.196 | 3.220 | -19.00/-19.13/-18.84 |
| 4/6 | lower_tool | plus_x_alternating_y_curve | static | 62 | 1.1 | -1.732/-2.305/+0.025 | 0.484 | -10.69/-19.08/0.86 |
| 5/6 | paint_path | plus_x_alternating_y_curve | +X | 261 | 22.8 | +0.015/-0.208/+0.228 | 2.496 | 1.01/0.79/1.23 |
| 6/6 | lift_tool | plus_x_alternating_y_curve | static | 61 | 0.2 | +1.713/-0.003/+2.183 | 0.101 | -7.08/-18.51/1.00 |

Per-joint max |error| (deg):
  [3] smooth_s_curve: upperArm_joint: 0.41, wrist3_joint: 0.39, wrist1_joint: 0.31
  [4] smooth_s_curve: wrist1_joint: 0.39, upperArm_joint: 0.33, foreArm_joint: 0.08
  [5] smooth_s_curve: wrist3_joint: 0.35, shoulder_joint: 0.32, foreArm_joint: 0.32
  [6] smooth_s_curve: wrist1_joint: 0.39, upperArm_joint: 0.33, foreArm_joint: 0.09
  [7] closed_circle: wrist3_joint: 0.35, foreArm_joint: 0.34, shoulder_joint: 0.33
  [8] closed_circle: wrist1_joint: 0.39, upperArm_joint: 0.33, foreArm_joint: 0.08
  [9] closed_circle: wrist3_joint: 0.35, shoulder_joint: 0.33, foreArm_joint: 0.33
  [10] closed_circle: wrist1_joint: 0.38, upperArm_joint: 0.33, foreArm_joint: 0.09
  [11] sine_squiggle: wrist3_joint: 0.36, shoulder_joint: 0.33, foreArm_joint: 0.09
  [12] sine_squiggle: wrist1_joint: 0.39, upperArm_joint: 0.33, foreArm_joint: 0.08
  [13] sine_squiggle: foreArm_joint: 0.34, wrist3_joint: 0.21, shoulder_joint: 0.20
  [14] sine_squiggle: wrist1_joint: 0.39, upperArm_joint: 0.33, foreArm_joint: 0.08
  [15] sharp_corners: wrist3_joint: 0.36, shoulder_joint: 0.33, foreArm_joint: 0.25
  [16] sharp_corners: wrist1_joint: 0.38, upperArm_joint: 0.33, foreArm_joint: 0.07
  [17] sharp_corners: wrist3_joint: 0.36, foreArm_joint: 0.33, shoulder_joint: 0.33
  [18] sharp_corners: wrist1_joint: 0.38, upperArm_joint: 0.33, foreArm_joint: 0.07
  [3] plus_y_retract_same_segment: foreArm_joint: 0.40, upperArm_joint: 0.23, wrist1_joint: 0.20
  [4] plus_y_retract_same_segment: wrist1_joint: 0.38, upperArm_joint: 0.33, foreArm_joint: 0.07
  [5] plus_y_retract_same_segment: foreArm_joint: 0.32, upperArm_joint: 0.18, wrist1_joint: 0.18
  [6] plus_y_retract_same_segment: wrist1_joint: 0.38, upperArm_joint: 0.32, foreArm_joint: 0.06
  [7] minus_y_extend_same_segment: wrist1_joint: 0.03, upperArm_joint: 0.02, shoulder_joint: 0.01
  [8] minus_y_extend_same_segment: wrist1_joint: 0.38, upperArm_joint: 0.32, foreArm_joint: 0.08
  [9] minus_y_extend_same_segment: foreArm_joint: 0.33, wrist1_joint: 0.18, upperArm_joint: 0.17
  [10] minus_y_extend_same_segment: wrist1_joint: 0.38, upperArm_joint: 0.33, foreArm_joint: 0.08
  [11] plus_x_control_same_segment: foreArm_joint: 0.34, upperArm_joint: 0.18, wrist1_joint: 0.17
  [12] plus_x_control_same_segment: wrist1_joint: 0.38, upperArm_joint: 0.32, foreArm_joint: 0.08
  [13] plus_x_control_same_segment: wrist3_joint: 0.35, shoulder_joint: 0.32, wrist1_joint: 0.08
  [14] plus_x_control_same_segment: wrist1_joint: 0.38, upperArm_joint: 0.32, foreArm_joint: 0.07
  [15] minus_x_control_same_segment: wrist1_joint: 0.01, upperArm_joint: 0.00, foreArm_joint: 0.00
  [16] minus_x_control_same_segment: wrist1_joint: 0.38, upperArm_joint: 0.33, foreArm_joint: 0.08
  [17] minus_x_control_same_segment: wrist3_joint: 0.35, shoulder_joint: 0.33, foreArm_joint: 0.07
  [18] minus_x_control_same_segment: wrist1_joint: 0.38, upperArm_joint: 0.33, foreArm_joint: 0.07
  [3] plus_y_then_minus_y: foreArm_joint: 0.40, upperArm_joint: 0.23, wrist1_joint: 0.20
  [4] plus_y_then_minus_y: wrist1_joint: 0.38, upperArm_joint: 0.32, foreArm_joint: 0.08
  [5] plus_y_then_minus_y: foreArm_joint: 0.32, upperArm_joint: 0.18, wrist1_joint: 0.18
  [6] plus_y_then_minus_y: wrist1_joint: 0.38, upperArm_joint: 0.32, foreArm_joint: 0.08
  [7] minus_y_then_plus_y: foreArm_joint: 0.32, upperArm_joint: 0.19, wrist1_joint: 0.16
  [8] minus_y_then_plus_y: wrist1_joint: 0.38, upperArm_joint: 0.32, foreArm_joint: 0.08
  [9] minus_y_then_plus_y: foreArm_joint: 0.34, upperArm_joint: 0.19, wrist1_joint: 0.18
  [10] minus_y_then_plus_y: wrist1_joint: 0.37, upperArm_joint: 0.32, foreArm_joint: 0.06
  [3] plus_x_alternating_y_curve: foreArm_joint: 0.42, wrist3_joint: 0.33, shoulder_joint: 0.31
  [4] plus_x_alternating_y_curve: wrist1_joint: 0.38, upperArm_joint: 0.32, foreArm_joint: 0.08
  [5] plus_x_alternating_y_curve: foreArm_joint: 0.33, wrist3_joint: 0.31, shoulder_joint: 0.29
  [6] plus_x_alternating_y_curve: wrist1_joint: 0.38, upperArm_joint: 0.32, foreArm_joint: 0.06

## Phase delay & normal oscillation
Instantaneous-direction normal error mean/min/max mm (n):
  [3] smooth_s_curve: -Y -3.118/-3.312/-2.714 (n=105); mixed -2.309/-3.126/-0.042 (n=551)
  [4] smooth_s_curve: mixed -0.052/-0.052/-0.052 (n=1)
  [5] smooth_s_curve: +X +0.053/-0.195/+0.198 (n=194); mixed -0.097/-0.426/+0.534 (n=55)
  [6] smooth_s_curve: mixed +0.028/+0.001/+0.054 (n=2)
  [7] closed_circle: mixed +0.109/-0.703/+0.216 (n=244)
  [8] closed_circle: mixed -0.008/-0.019/+0.002 (n=2)
  [9] closed_circle: +X +0.035/-0.070/+0.193 (n=28); +Y +0.126/+0.085/+0.200 (n=60); -X -0.055/-0.231/+0.055 (n=27); -Y -0.132/-0.168/-0.102 (n=55); mixed -0.007/-0.183/+0.185 (n=234)
  [10] closed_circle: -Y -0.032/-0.056/-0.008 (n=2)
  [11] sine_squiggle: +X +0.002/-0.262/+0.145 (n=32)
  [12] sine_squiggle: (no directed motion)
  [13] sine_squiggle: +X +0.012/-0.169/+0.133 (n=15); +Y +0.119/-0.006/+0.191 (n=268); -Y -0.132/-0.171/-0.100 (n=256); mixed -0.013/-0.331/+0.329 (n=230)
  [14] sine_squiggle: +Y -0.004/-0.004/-0.004 (n=1)
  [15] sharp_corners: mixed +0.032/-0.207/+0.080 (n=315)
  [16] sharp_corners: mixed -0.015/-0.026/-0.004 (n=2)
  [17] sharp_corners: +X +0.057/-0.100/+0.279 (n=67); +Y +0.116/+0.018/+0.244 (n=234); mixed -0.054/-0.235/+0.245 (n=403)
  [18] sharp_corners: (no directed motion)
  [3] plus_y_retract_same_segment: -Y +0.000/+0.000/+0.000 (n=1); mixed -0.146/-0.237/+0.009 (n=275)
  [4] plus_y_retract_same_segment: mixed +0.040/+0.016/+0.064 (n=2)
  [5] plus_y_retract_same_segment: +X +0.123/-0.070/+0.317 (n=2); +Y +0.139/+0.084/+0.337 (n=101)
  [6] plus_y_retract_same_segment: -Y -0.025/-0.043/-0.007 (n=2)
  [7] minus_y_extend_same_segment: +X -0.011/-0.133/+0.110 (n=2)
  [8] minus_y_extend_same_segment: (no directed motion)
  [9] minus_y_extend_same_segment: -Y -0.111/-0.172/+0.126 (n=100); mixed +0.052/-0.019/+0.123 (n=2)
  [10] minus_y_extend_same_segment: +Y +0.009/-0.030/+0.048 (n=2)
  [11] plus_x_control_same_segment: +Y +0.096/-0.379/+0.169 (n=175)
  [12] plus_x_control_same_segment: mixed -0.051/-0.094/-0.007 (n=2)
  [13] plus_x_control_same_segment: +X +0.038/-0.057/+0.557 (n=53); mixed +0.247/-0.057/+0.550 (n=2)
  [14] plus_x_control_same_segment: (no directed motion)
  [15] minus_x_control_same_segment: (no directed motion)
  [16] minus_x_control_same_segment: (no directed motion)
  [17] minus_x_control_same_segment: -X +0.000/-0.020/+0.044 (n=53)
  [18] minus_x_control_same_segment: +X +0.015/+0.007/+0.023 (n=2)
  [3] plus_y_then_minus_y: mixed +0.146/+0.000/+0.204 (n=81)
  [4] plus_y_then_minus_y: -Y -0.031/-0.059/-0.003 (n=2)
  [5] plus_y_then_minus_y: +Y +0.113/+0.054/+0.418 (n=89); -Y -0.105/-0.223/+0.075 (n=90); mixed +0.211/+0.004/+0.419 (n=2)
  [6] plus_y_then_minus_y: +Y +0.027/+0.010/+0.045 (n=2)
  [7] minus_y_then_plus_y: +Y +0.096/-0.475/+0.183 (n=94)
  [8] minus_y_then_plus_y: -Y -0.018/-0.036/-0.000 (n=2)
  [9] minus_y_then_plus_y: +Y +0.116/-0.092/+0.261 (n=89); -Y -0.092/-0.156/+0.337 (n=89); mixed +0.337/+0.337/+0.337 (n=1)
  [10] minus_y_then_plus_y: mixed -0.010/-0.032/+0.011 (n=2)
  [3] plus_x_alternating_y_curve: mixed +0.081/-0.028/+0.196 (n=101)
  [4] plus_x_alternating_y_curve: mixed -0.018/-0.048/+0.011 (n=2)
  [5] plus_x_alternating_y_curve: +X +0.016/-0.091/+0.144 (n=14); mixed +0.015/-0.208/+0.228 (n=224)
  [6] plus_x_alternating_y_curve: -Y -0.002/-0.003/-0.001 (n=2)

Per-segment delay and canvas-normal oscillation:
  [3] smooth_s_curve: delay n/a; normal pp seg 3.31 mm, per-cycle mean/max 3.31/3.31 mm (0 cyc), rms 2.471 mm
  [4] smooth_s_curve: delay median 88 ms (wrist3_joint:160, foreArm_joint:96); normal pp seg 3.86 mm, per-cycle mean/max 3.86/3.86 mm (0 cyc), rms 2.885 mm
  [5] smooth_s_curve: delay median 96 ms (upperArm_joint:96, wrist1_joint:96); normal pp seg 0.96 mm, per-cycle mean/max 0.96/0.96 mm (1 cyc), rms 0.126 mm
  [6] smooth_s_curve: delay median 88 ms (wrist3_joint:192, foreArm_joint:96); normal pp seg 3.63 mm, per-cycle mean/max 3.63/3.63 mm (0 cyc), rms 2.839 mm
  [7] closed_circle: delay median 96 ms (shoulder_joint:96, wrist3_joint:96); normal pp seg 1.06 mm, per-cycle mean/max 1.06/1.06 mm (0 cyc), rms 0.166 mm
  [8] closed_circle: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 3.19 mm, per-cycle mean/max 3.19/3.19 mm (0 cyc), rms 2.511 mm
  [9] closed_circle: delay median 96 ms (wrist2_joint:128, upperArm_joint:96); normal pp seg 0.43 mm, per-cycle mean/max 0.43/0.43 mm (1 cyc), rms 0.122 mm
  [10] closed_circle: delay median 80 ms (upperArm_joint:80, wrist1_joint:80); normal pp seg 3.07 mm, per-cycle mean/max 3.07/3.07 mm (0 cyc), rms 2.460 mm
  [11] sine_squiggle: delay median 96 ms (upperArm_joint:112, foreArm_joint:96); normal pp seg 0.42 mm, per-cycle mean/max 0.42/0.42 mm (0 cyc), rms 0.133 mm
  [12] sine_squiggle: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 3.18 mm, per-cycle mean/max 3.18/3.18 mm (0 cyc), rms 2.496 mm
  [13] sine_squiggle: delay median 96 ms (wrist2_joint:160, upperArm_joint:96); normal pp seg 0.66 mm, per-cycle mean/max 0.60/0.64 mm (3 cyc), rms 0.134 mm
  [14] sine_squiggle: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 3.09 mm, per-cycle mean/max 3.09/3.09 mm (0 cyc), rms 2.471 mm
  [15] sharp_corners: delay median 96 ms (upperArm_joint:96, foreArm_joint:96); normal pp seg 0.35 mm, per-cycle mean/max 0.35/0.35 mm (0 cyc), rms 0.050 mm
  [16] sharp_corners: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 2.84 mm, per-cycle mean/max 2.84/2.84 mm (0 cyc), rms 2.289 mm
  [17] sharp_corners: delay median 96 ms (wrist2_joint:208, upperArm_joint:96); normal pp seg 0.51 mm, per-cycle mean/max 0.48/0.51 mm (3 cyc), rms 0.116 mm
  [18] sharp_corners: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 2.60 mm, per-cycle mean/max 2.60/2.60 mm (0 cyc), rms 2.150 mm
  [3] plus_y_retract_same_segment: delay n/a; normal pp seg 0.25 mm, per-cycle mean/max 0.25/0.25 mm (0 cyc), rms 0.149 mm
  [4] plus_y_retract_same_segment: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 3.36 mm, per-cycle mean/max 3.36/3.36 mm (0 cyc), rms 2.587 mm
  [5] plus_y_retract_same_segment: delay n/a; normal pp seg 0.53 mm, per-cycle mean/max 0.53/0.53 mm (0 cyc), rms 0.152 mm
  [6] plus_y_retract_same_segment: delay median 80 ms (upperArm_joint:80, wrist1_joint:80); normal pp seg 3.05 mm, per-cycle mean/max 3.05/3.05 mm (0 cyc), rms 2.426 mm
  [7] minus_y_extend_same_segment: delay n/a; normal pp seg 0.26 mm, per-cycle mean/max 0.26/0.26 mm (0 cyc), rms 0.140 mm
  [8] minus_y_extend_same_segment: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 3.08 mm, per-cycle mean/max 3.08/3.08 mm (0 cyc), rms 2.496 mm
  [9] minus_y_extend_same_segment: delay n/a; normal pp seg 0.30 mm, per-cycle mean/max 0.30/0.30 mm (0 cyc), rms 0.122 mm
  [10] minus_y_extend_same_segment: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 3.38 mm, per-cycle mean/max 3.38/3.38 mm (0 cyc), rms 2.620 mm
  [11] plus_x_control_same_segment: delay median 96 ms (shoulder_joint:96, wrist3_joint:96); normal pp seg 0.55 mm, per-cycle mean/max 0.55/0.55 mm (0 cyc), rms 0.136 mm
  [12] plus_x_control_same_segment: delay median 88 ms (shoulder_joint:128, foreArm_joint:96); normal pp seg 3.01 mm, per-cycle mean/max 3.01/3.01 mm (0 cyc), rms 2.444 mm
  [13] plus_x_control_same_segment: delay median 96 ms (upperArm_joint:96, foreArm_joint:96); normal pp seg 0.92 mm, per-cycle mean/max 0.92/0.92 mm (0 cyc), rms 0.179 mm
  [14] plus_x_control_same_segment: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 2.89 mm, per-cycle mean/max 2.89/2.89 mm (0 cyc), rms 2.299 mm
  [15] minus_x_control_same_segment: delay n/a; normal pp seg 0.01 mm, per-cycle mean/max 0.01/0.01 mm (0 cyc), rms 0.032 mm
  [16] minus_x_control_same_segment: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 3.00 mm, per-cycle mean/max 3.00/3.00 mm (0 cyc), rms 2.320 mm
  [17] minus_x_control_same_segment: delay median 96 ms (upperArm_joint:112, foreArm_joint:96); normal pp seg 0.08 mm, per-cycle mean/max 0.08/0.08 mm (0 cyc), rms 0.020 mm
  [18] minus_x_control_same_segment: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 2.95 mm, per-cycle mean/max 2.95/2.95 mm (0 cyc), rms 2.415 mm
  [3] plus_y_then_minus_y: delay median 96 ms (upperArm_joint:96, wrist1_joint:96); normal pp seg 0.20 mm, per-cycle mean/max 0.20/0.20 mm (0 cyc), rms 0.142 mm
  [4] plus_y_then_minus_y: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 2.81 mm, per-cycle mean/max 2.81/2.81 mm (0 cyc), rms 2.247 mm
  [5] plus_y_then_minus_y: delay median 96 ms (shoulder_joint:96, upperArm_joint:96); normal pp seg 0.67 mm, per-cycle mean/max 0.67/0.67 mm (1 cyc), rms 0.126 mm
  [6] plus_y_then_minus_y: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 2.74 mm, per-cycle mean/max 2.74/2.74 mm (0 cyc), rms 2.255 mm
  [7] minus_y_then_plus_y: delay median 96 ms (upperArm_joint:96); normal pp seg 0.66 mm, per-cycle mean/max 0.66/0.66 mm (0 cyc), rms 0.168 mm
  [8] minus_y_then_plus_y: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 2.58 mm, per-cycle mean/max 2.58/2.58 mm (0 cyc), rms 2.098 mm
  [9] minus_y_then_plus_y: delay median 96 ms (shoulder_joint:96, upperArm_joint:96); normal pp seg 0.54 mm, per-cycle mean/max 0.54/0.54 mm (1 cyc), rms 0.131 mm
  [10] minus_y_then_plus_y: delay median 80 ms (upperArm_joint:80, foreArm_joint:80); normal pp seg 2.50 mm, per-cycle mean/max 2.50/2.50 mm (0 cyc), rms 2.069 mm
  [3] plus_x_alternating_y_curve: delay n/a; normal pp seg 0.22 mm, per-cycle mean/max 0.22/0.22 mm (1 cyc), rms 0.104 mm
  [4] plus_x_alternating_y_curve: delay median 96 ms (wrist3_joint:128, shoulder_joint:96); normal pp seg 2.33 mm, per-cycle mean/max 2.33/2.33 mm (0 cyc), rms 1.896 mm
  [5] plus_x_alternating_y_curve: delay median 96 ms (wrist2_joint:160, upperArm_joint:96); normal pp seg 0.44 mm, per-cycle mean/max 0.40/0.44 mm (2 cyc), rms 0.120 mm
  [6] plus_x_alternating_y_curve: delay median 80 ms (foreArm_joint:96, upperArm_joint:80); normal pp seg 2.19 mm, per-cycle mean/max 2.19/2.19 mm (0 cyc), rms 1.846 mm
Phase 2B tracking gate: FAIL (delay median <30 ms: FAIL, delay p95 <50 ms: FAIL, |normal| <=0.25 mm: FAIL)
  delay median/p95 96/128 ms; worst |normal| 3.65 mm; worst per-cycle pp 3.86 mm (the per-cycle pp is the objective proxy for the operator's 'visible wrist oscillation' check).

## ServoJ timing (aubo_servoj_diag)
config: t=0.008 s, gain=200, window=250 cycles
297 reports over 74250 cycles; 125.0 Hz effective loop (100.0% of 125 Hz configured)
period ms: mean 8.00, min 3.12, max 534.90, p95 11.77 / p99 12.65 (worst window)
servoJoint RPC ms: mean 2.11, max 5.48; whole Servoj ms: mean 4.29, max 13.60
late cycles: 93 (worst run 1); queue-full: 91 events, 91 retries, 1041.5 ms blocked
return codes: ok 74250, busy 0, bad 0, inval 0, ign 0, other 0; exceptions 0
log warnings: mismatch 1, rc 0, queue-full 0; fault latched: no
Phase 2B timing gate: FAIL (rate >= 95%: ok, no queue-full: FAIL, no non-OK rc/exc: ok, no timing fault: ok)
  (joint-delay gate <30 ms median / <50 ms p95 is assessed separately from controller_state cross-correlation.)
