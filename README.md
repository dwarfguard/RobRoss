# RobRoss / R.O.B Ross

**Status:** Active prototype development  
**Current target:** Aubo i5 drawing a monochrome Mondrian-style image on A4 paper

RobRoss is a robot-arm art project. Demo v1 focuses on one practical goal:
reliably executing a pre-generated pen path on a calibrated physical surface.
Paint, color changes, audience interaction, and real-time image generation are
outside the current milestone.

RobRoss 是一个机器人绘画项目。当前 Demo v1 的目标是让 Aubo i5 在经过标定的
A4 纸面上稳定执行预生成的黑色线条路径。颜料、换色、观众交互和实时图像生成
不属于当前阶段。

## System Overview

```text
configs/*.json
    |
    v
Mondrian artwork generator
    |
    +-- output/painting_plan.json
    +-- output/mondrian_preview.svg
    |
    v
Path generator and validator
    |
    +-- output/painting_paths.json
    +-- output/path_preview.svg
    +-- output/path_animation.svg
    |
    v
robross_painter -> MoveIt -> Aubo i5
```

Generated coordinates use millimeters with the origin at the paper's top-left,
`x` pointing right, and `y` pointing down. `painting_paths.json` is an
intermediate command format, not motor-control output. See the
[path format reference](docs/painting-paths-format.md).

## Start Here

| Goal | Guide |
| --- | --- |
| Generate artwork and paths | [Mondrian pipeline](Image_Process/mondrian/README.md) |
| Understand the path schema | [Path format](docs/painting-paths-format.md) |
| Build and run in RViz | [ROS 2 painter](ros2/robross_painter/README.md) |
| Prepare a real-arm session | [Hardware preflight](ros2/robross_painter/PREFLIGHT.md) |
| Review prototype requirements | [Prototype v1](docs/Rob_Ross_Prototype_v1.md) |
| Work with CAD assemblies | [CAD assets](CAD/README.md) |

The active artwork profile is `configs/demo_v1_a4_pen.json`. The
`mondrian_12x12_paint.json` profile preserves the older color-canvas behavior
for development and is not the Demo v1 hardware target.

## Reproduce The ROS 2 Workspace

The supported baseline is Ubuntu 22.04 with ROS 2 Humble. Install ROS 2 first,
then install the workspace tools and MoveIt packages:

```bash
sudo apt update
sudo apt install \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
sudo rosdep init 2>/dev/null || true
rosdep update
```

Create the workspace from the `sai` branch. The repository manifest imports the
RobRoss Aubo branch, and that repository pins the description submodule:

```bash
mkdir -p ~/robross_aubo_ws/src
git clone --branch sai https://github.com/dwarfguard/RobRoss.git \
  ~/robross_aubo_ws/src/RobRoss
vcs import ~/robross_aubo_ws/src < ~/robross_aubo_ws/src/RobRoss/ros2/robross_aubo.repos
git -C ~/robross_aubo_ws/src/aubo_ros2_driver submodule update --init --recursive
source /opt/ros/humble/setup.bash
cd ~/robross_aubo_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --event-handlers console_direct+
source install/setup.bash
```

Before approving a hardware build, record all three source revisions. Use the
same revisions on every computer; do not assume a moving branch still contains
the approved build:

```bash
git -C src/RobRoss rev-parse HEAD
git -C src/aubo_ros2_driver rev-parse HEAD
git -C src/aubo_ros2_driver/aubo_description rev-parse HEAD
```

On a reproduction computer, check out the recorded RobRoss and driver SHAs,
then run `git submodule update --init --recursive` again so the recorded driver
selects its matching description SHA. Network access is required during the
first driver build to download Aubo SDK `0.24.1-rc.3+318754d`. Do not copy an
existing `build/` or `install/` directory to another computer.

```bash
git -C src/RobRoss checkout <recorded-robross-sha>
git -C src/aubo_ros2_driver checkout <recorded-driver-sha>
git -C src/aubo_ros2_driver submodule update --init --recursive
test "$(git -C src/aubo_ros2_driver/aubo_description rev-parse HEAD)" = \
  "<recorded-description-sha>"
```

Run the tests before using the workspace:

```bash
colcon test --packages-select \
  aubo_description aubo_moveit_config aubo_msgs aubo_ros2_driver \
  robross_painter ros_joints_plan
colcon test-result --verbose
```

Continue with the [painter guide](ros2/robross_painter/README.md) for the
fake-hardware and RViz launch sequence.

> **Real hardware:** Never rely on the painter's default calibration; it is an
> RViz-only virtual wall. A real-arm launch must explicitly provide a reviewed
> hardware profile and a freshly taught canvas pose. Complete the
> [hardware preflight](ros2/robross_painter/PREFLIGHT.md) before enabling motion.

## Repository Layout

```text
configs/                         Artwork and path-generation profiles
Image_Process/mondrian/          Artwork, path, preview, and test-line tools
ros2/robross_painter/            MoveIt path executor and canvas teaching tool
docs/                            Requirements and path-format references
CAD/                             Tool, canvas, and paint-holder models
output/                          Generated plans, paths, and previews
```

## Current Scope

| Area | Demo v1 decision |
| --- | --- |
| Surface | A4 portrait paper, 210 mm x 297 mm |
| Tool | Spring-loaded pen |
| Artwork | Preset monochrome Mondrian-style lines |
| Planning | Pre-generated paths executed through MoveIt |
| Robot | Aubo i5 |

Robot calibration, tool geometry, canvas pose, collision geometry, and motion
limits belong in `ros2/robross_painter`, not in artwork profiles.

## Contributing

Keep artwork generation, path generation, validation, and robot execution
separate. Update the relevant guide when behavior changes. Coding agents should
also read [AGENTS.md](AGENTS.md).

Product ideas and earlier design discussion are retained in
[docs/Rob_Ross_Discuss.md](docs/Rob_Ross_Discuss.md).
