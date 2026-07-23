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
    +-- output/<config-name>/painting_plan.json
    +-- output/<config-name>/mondrian_preview.svg
    |
    v
Path generator and validator
    |
    +-- output/<config-name>/painting_paths.json
    +-- output/<config-name>/path_preview.svg
    +-- output/<config-name>/path_animation.svg
    |
    v
robross_painter -> MoveIt -> Aubo i5
```

Generated coordinates use millimeters with the origin at the paper's top-left,
`x` pointing right, and `y` pointing down. `painting_paths.json` is an
intermediate command format, not motor-control output. See the
[path format reference](docs/painting-paths-format.md).

Each config profile writes into its own `output/<config-name>/` subfolder (the
name matches the config's filename, minus `.json`) so different profiles never
clobber each other's output. After generating one or more configs, run
`python3 generate_output_gallery.py` and open the resulting `output/index.html`
in a browser for a quick side-by-side preview of every generated run (previews,
validation status, path/stroke counts) instead of opening files one by one.

## Start Here

| Goal | Guide |
| --- | --- |
| Generate artwork and paths | [Mondrian pipeline](Image_Process/mondrian/README.md) |
| Trace edges of a source image | [Sketch route](Image_Process/sketch/README.md) |
| Turn a photo into Mondrian-style fill art | [Image-to-Mondrian route](Image_Process/image_to_mondrian/README.md) |
| Turn a photo into Mondrian-style art (AI restyle) | [Gemini Mondrian route](Image_Process/gemini_mondrian/README.md) |
| Trace a clean line-art/technical illustration | [line_art route](Image_Process/line_art/README.md) |
| Browse all generated runs | Run `python3 generate_output_gallery.py` → `output/index.html` |
| Understand the path schema | [Path format](docs/painting-paths-format.md) |
| Build and run in RViz | [ROS 2 painter](ros2/robross_painter/README.md) |
| Prepare a real-arm session | [Hardware preflight](ros2/robross_painter/PREFLIGHT.md) |
| Review prototype requirements | [Prototype v1](docs/Rob_Ross_Prototype_v1.md) |
| Work with CAD assemblies | [CAD assets](CAD/README.md) |

The active artwork profile is `configs/demo_v1_a4_pen.json`. The
`mondrian_12x12_paint.json` profile preserves the older color-canvas behavior
for development and is not the Demo v1 hardware target.

## ROS 2 Workspace

The painter uses ROS 2 Humble, MoveIt 2, and the RobRoss-maintained Aubo driver
fork. Create a workspace with the repository and its pinned dependencies:

```bash
mkdir -p ~/robross_aubo_ws/src
git clone https://github.com/dwarfguard/RobRoss.git ~/robross_aubo_ws/src/RobRoss
vcs import ~/robross_aubo_ws/src < ~/robross_aubo_ws/src/RobRoss/ros2/robross_aubo.repos
git -C ~/robross_aubo_ws/src/aubo_ros2_driver submodule update --init --recursive
source /opt/ros/humble/setup.bash
cd ~/robross_aubo_ws
colcon build
source install/setup.bash
```

Prerequisites:

- ROS 2 Humble, MoveIt 2, `colcon`, and `python3-vcstool`.
- Network access during the first driver build.
- The [`robross-fixes` Aubo driver](https://github.com/dwarfguard/aubo_ros2_driver/tree/robross-fixes), including its robot-calibration guidance for real hardware.

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
output/                          Generated plans, paths, and previews (one subfolder per config)
generate_output_gallery.py       Builds output/index.html, a static preview of every generated run
webapp/                          Optional local control panel: upload a photo, run a route, browse the result
```

`webapp/` is an optional add-on (needs `pip install flask`) — see
[webapp/README.md](webapp/README.md). It's a thin wrapper around the same
CLI scripts described above, not a separate implementation.

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
