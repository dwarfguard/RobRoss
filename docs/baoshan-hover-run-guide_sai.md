# Baoshan Supervised Hover Run

This procedure executes only `output/baoshan.json` against the hover-only canvas. It does not run
the direction, reversal, or curve test paths. The configured 20 mm canvas shift provides
approximately 19 mm of physical paper clearance; this procedure does not authorize paper contact.

Keep an operator at the e-stop. Stop any existing driver, MoveIt, executor, and rosbag processes
before starting. Never run two real-hardware drivers simultaneously.

## 0. Create a Fresh Run

Run this once before opening the four run terminals:

```bash
unset ROS_DOMAIN_ID ROBROSS_FAKE_HEALTH
cd ~/robross_aubo_ws
source install/setup.bash

export RUN_ID="robross_baoshan_$(date -u +%Y%m%dT%H%M%SZ)"
export RUN_SESSION="$HOME/$RUN_ID"
mkdir -p "$RUN_SESSION/cartesian_failures"

{
  printf 'export ROBOT_IP=%q\n' "192.168.32.101"
  printf 'export AUBO_TYPE=%q\n' "aubo_i5_calibrated"
  printf 'export RUN_SESSION=%q\n' "$RUN_SESSION"
  printf 'export ROBROSS_SESSION_ID=%q\n' "$RUN_ID"
  printf 'export ROBROSS_REPO=%q\n' "$PWD/src/RobRoss"
  printf 'export EXPECTED_DRIVER_COMMIT=%q\n' \
    "71996f8aca5b3510f426423daa5e5c6bc4d7b827"
  printf 'export EXPECTED_CONTROLLER_REPOSITORY=%q\n' \
    "https://github.com/ros-controls/ros2_controllers.git"
  printf 'export EXPECTED_CONTROLLER_COMMIT=%q\n' \
    "81284e1a4e14ec5158d3963903e2cf060d951f20"
} > /tmp/robross_baoshan_run.env

printf 'Run directory: %s\n' "$RUN_SESSION"
```

Run this setup at the beginning of each terminal:

```bash
unset ROS_DOMAIN_ID ROBROSS_FAKE_HEALTH
cd ~/robross_aubo_ws
source install/setup.bash
source /tmp/robross_baoshan_run.env
```

## 1. Start Rosbag

In Terminal 1, start recording before starting the driver:

```bash
export BAG_DIR="$RUN_SESSION/baoshan_125hz_008s"

test ! -e "$BAG_DIR" || {
  echo "Bag path already exists"
  exit 1
}

ros2 bag record -o "$BAG_DIR" \
  /aubo_driver/health \
  /joint_trajectory_controller/controller_state \
  /joint_states \
  /parameter_events \
  /robot_description \
  /rosout \
  /tf \
  /tf_static \
  /robross_markers
```

## 2. Start the Driver

In Terminal 2:

```bash
export AUBO_SERVOJ_TELEMETRY_CSV="$RUN_SESSION/baoshan_servoj_calls.csv"

test ! -e "$AUBO_SERVOJ_TELEMETRY_CSV" || {
  echo "Telemetry file already exists"
  exit 1
}

test ! -e "$AUBO_SERVOJ_TELEMETRY_CSV.partial" || {
  echo "Partial telemetry file already exists"
  exit 1
}

./src/RobRoss/start_hardware_stack.sh \
  --session-dir "$RUN_SESSION" \
  --session-id "$ROBROSS_SESSION_ID" -- \
  ros2 launch aubo_ros2_driver aubo_control.launch.py \
    aubo_type:="$AUBO_TYPE" \
    robot_ip:="$ROBOT_IP" \
    use_fake_hardware:=false \
    controllers_file:=aubo_controllers_125hz.yaml \
    servoj_time:=0.008 \
    rtde_state_rate:=200 \
    rtde_state_max_age:=0.05
```

Wait until the driver reports healthy joint states and active controllers.

## 3. Start MoveIt

In Terminal 3:

```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
  aubo_type:="$AUBO_TYPE"
```

Wait until MoveIt is ready before starting the executor.

## 4. Execute Baoshan

Before continuing, physically confirm hover clearance, a clear reach sphere, expected starting
geometry, and an operator at the e-stop.

In Terminal 4:

```bash
set -euo pipefail

read -r -p "Ready to execute baoshan.json in hover? Type yes: " ready
test "$ready" = "yes" || exit 1

./src/RobRoss/start_hardware_stack.sh \
  --session-dir "$RUN_SESSION" \
  --session-id "$ROBROSS_SESSION_ID" -- \
  ros2 launch robross_painter paint.launch.py \
    aubo_type:="$AUBO_TYPE" \
    require_qualified_build:=true \
    expected_driver_commit:="$EXPECTED_DRIVER_COMMIT" \
    expected_controller_repository:="$EXPECTED_CONTROLLER_REPOSITORY" \
    expected_controller_commit:="$EXPECTED_CONTROLLER_COMMIT" \
    calibration_file:="$HOME/hardware_hover_a4.yaml" \
    canvas_file:="$HOME/canvas_hover_20mm.yaml" \
    cartesian_failure_artifact_dir:="$RUN_SESSION/cartesian_failures" \
    paths_file:="$ROBROSS_REPO/output/baoshan.json" 2>&1 | \
  tee "$RUN_SESSION/baoshan_executor.log"
```
./src/RobRoss/start_hardware_stack.sh \
  --session-dir "$RUN_SESSION" \
  --session-id "$ROBROSS_SESSION_ID" -- \
  ros2 launch robross_painter paint.launch.py \
    aubo_type:="$AUBO_TYPE" \
    calibration_file:="$HOME/hardware_4k_landscape.yaml" \
    canvas_file:="$HOME/canvas_4k_landscape_hover_20mm.yaml" \
    cartesian_failure_artifact_dir:="$RUN_SESSION/cartesian_failures" \
    paths_file:="$ROBROSS_REPO/output/baoshan_4k.json"

Stop immediately for loss of hover clearance, unexpected geometry, visible wrist oscillation,
queue-full warnings, nonzero ServoJ return codes, driver faults, or timing faults.

## Shutdown and Preserve Evidence

After the executor finishes:

1. Stop rosbag with `Ctrl-C`.
2. Stop MoveIt with `Ctrl-C`.
3. Stop the driver with `Ctrl-C` and wait for it to exit completely.
4. Do not launch another driver using this run directory.

Verify that the full-rate ServoJ telemetry finalized:

```bash
source /tmp/robross_baoshan_run.env

test -f "$RUN_SESSION/baoshan_servoj_calls.csv"
test ! -e "$RUN_SESSION/baoshan_servoj_calls.csv.partial"

tail -n 1 "$RUN_SESSION/baoshan_servoj_calls.csv" | \
  tee "$RUN_SESSION/baoshan_servoj_footer.txt"

grep -F "status=complete" "$RUN_SESSION/baoshan_servoj_footer.txt"
grep -F "dropped=0" "$RUN_SESSION/baoshan_servoj_footer.txt"
```

Do not delete, rename, truncate, or reuse the finalized telemetry file or bag directory.
