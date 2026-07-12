# RobRoss / R.O.B Ross

**Status:** Active prototype development  
**Current focus:** Demo v1 — A4 pen-on-paper Mondrian line drawing for Aubo i5 hardware testing  
**Audience:** Human collaborators, software contributors, hardware contributors, and LLM coding agents

RobRoss is a robot-arm art project. The long-term product vision is an interactive installation where a robot paints simple artwork for an audience and creates a finished souvenir canvas. The current prototype is intentionally much smaller: prove that the robot can reliably draw a simple preprocessed artwork on A4 paper before adding paint, color changes, user interaction, or real-time AI.

RobRoss 是一个机器人绘画项目。长期目标是做成可供观众观看和互动的绘画装置；当前第一版原型则故意保持简单：先验证机器人能否在 A4 纸上稳定完成一幅预处理好的简单图案，再考虑颜料、换色、用户交互或实时 AI。

---

## Current Prototype: Demo v1 / 当前原型

Demo v1 is the active development target.

| Area | Current decision |
| --- | --- |
| Canvas / Paper | A4 portrait paper, 210 mm x 297 mm |
| Tool | Pen on paper |
| Color | Monochrome, black lines only |
| Artwork | Preset Mondrian-inspired geometric line drawing |
| Pathing | Preprocessed movement instructions, not real-time AI |
| Robot | Aubo i5 hardware testing target |
| Robot code | ROS 2 adapter in `ros2/robross_painter` executes path files through MoveIt on Aubo i5 |

Demo v1 is meant to answer one practical question:

> Can the robot reliably follow preprocessed drawing paths on a real physical surface?

第一版原型的核心目标不是完整艺术效果，而是验证机械臂能否在真实纸面上稳定执行预处理路径。

### Demo v1 non-goals / 第一版暂不做

- No live color mixing.
- No acrylic or oil paint for the first hardware test.
- No user-uploaded custom images.
- No real-time computer vision correction.
- No AI-generated robot motion during execution.
- No direct Aubo i5 motor-control output from the current path files.

这些功能未来可能有价值，但不属于当前第一版硬件测试范围。

---

## Long-Term Product Vision / 长期产品方向

The broader product idea is a public-facing robot painting installation for commercial spaces such as malls, cafés, events, and experience-based retail environments. A user may eventually choose from prepared artwork options, watch the robot paint, and take the finished work home as a souvenir.

长期愿景是让机械臂在商场、咖啡店、活动现场等公共消费空间中作画，吸引观众停留，并让用户带走一幅具有纪念意义的作品。

Possible future features include:

- 12-inch canvas painting.
- Acrylic paint or other paint media.
- Multiple colors and tool changes.
- Dedicated brushes or sponge tools.
- More personalized styles, such as simplified portrait or “blob” artwork.
- User-facing artwork selection.
- Maintenance and operator interface.
- Robot calibration and execution software.

These are future directions, not active Demo v1 requirements.

---

## Software Pipeline / 软件流程

The current software pipeline produces robot-style drawing instructions from a generated Mondrian-style layout.

```text
Config profile
  ↓
Image_Process/mondrian/mondrian_generator.py
  ↓
output/painting_plan.json
  ↓
Image_Process/mondrian/generate_painting_paths.py
  ↓
output/painting_paths.json
  ↓
ros2/robross_painter
  ↓
Robot motion
```

Important: `painting_paths.json` is **not** direct robot motor code. It is an intermediate representation using millimeter coordinates and abstract commands such as `move_to`, `lower_tool`, `paint_stroke`, and `lift_tool`. The ROS 2 adapter translates those commands into MoveIt motion.

---

## Config Profiles / 配置文件

The pipeline is config-driven. Each config profile defines the canvas, artwork mode, path/tool settings, and output filenames.

| Config | Purpose |
| --- | --- |
| `configs/demo_v1_a4_pen.json` | Active Demo v1 profile: A4 paper, monochrome line drawing, 1 mm pen settings. |
| `configs/mondrian_12x12_paint.json` | Legacy/future profile: 12-inch square canvas, colored Mondrian blocks, paint/brush-like settings. |

Use the A4 pen profile unless you are intentionally testing the older 12-inch color behavior.

---

## Quick Start / 快速运行

Generate the current Demo v1 A4 drawing plan:

```bash
python3 Image_Process/mondrian/mondrian_generator.py --config configs/demo_v1_a4_pen.json --seed 123
```

Generate path commands from that plan:

```bash
python3 Image_Process/mondrian/generate_painting_paths.py --config configs/demo_v1_a4_pen.json
```

Review the outputs:

```text
output/mondrian_preview.svg
output/painting_plan.json
output/path_preview.svg
output/path_animation.svg
output/painting_paths.json
```

`path_animation.svg` is an animated version of the path preview —
strokes draw themselves in execution order with travel moves and a tool
marker. Open it in a web browser (reload to replay).

Generate the single 50 mm first-contact test line (see the hardware
checklist) in the same path file format:

```bash
python3 Image_Process/mondrian/generate_test_line.py
```

Run the unit tests:

```bash
python3 -m unittest discover Image_Process/mondrian/tests
```

For the legacy 12-inch colored profile:

```bash
python3 Image_Process/mondrian/mondrian_generator.py --config configs/mondrian_12x12_paint.json --seed 123
python3 Image_Process/mondrian/generate_painting_paths.py --config configs/mondrian_12x12_paint.json
```

Always run both scripts with the same config profile. Mixing profiles can produce confusing or invalid results.

---

## ROS 2 Aubo Setup / ROS 2 遨博运行环境

The robot execution path uses this repo plus the RobRoss-maintained Aubo driver fork:

```text
RobRoss/ros2/robross_painter
github.com/dwarfguard/aubo_ros2_driver branch robross-fixes
github.com/dwarfguard/aubo_description branch robross-fixes (submodule)
```

Prerequisites on the target machine:

- ROS 2 Humble installed and sourced from `/opt/ros/humble/setup.bash`.
- MoveIt 2 and standard ROS build tools available.
- `python3-vcstool` available for `vcs import`.
- Network access for the Aubo driver CMake dependency download on first build.

Create a fresh workspace:

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

Generate the Demo v1 path files:

```bash
cd ~/robross_aubo_ws/src/RobRoss
python3 Image_Process/mondrian/mondrian_generator.py --config configs/demo_v1_a4_pen.json --seed 123
python3 Image_Process/mondrian/generate_painting_paths.py --config configs/demo_v1_a4_pen.json
python3 Image_Process/mondrian/generate_test_line.py
```

Run against fake hardware in three terminals from `~/robross_aubo_ws`.

Terminal 1, controllers:

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_i5 \
  use_fake_hardware:=true
```

Terminal 2, MoveIt and RViz:

```bash
source install/setup.bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

Terminal 3, execute the generated path:

```bash
source install/setup.bash
export ROBROSS_REPO=$PWD/src/RobRoss
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=$ROBROSS_REPO/output/painting_paths.json
```

For the first contact test, use the single 50 mm line path instead:

```bash
ros2 launch robross_painter paint.launch.py \
  aubo_type:=aubo_i5 \
  paths_file:=$ROBROSS_REPO/output/test_line_paths.json
```

For real hardware, replace fake hardware with the robot IP and update the
calibration YAML before allowing paper contact:

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py \
  aubo_type:=aubo_i5 \
  robot_ip:=<robot-ip> \
  use_fake_hardware:=false
```

Read `docs/hardware-test-checklist.md` before real robot testing.

---

## Repository Map / 文件结构

```text
configs/
  demo_v1_a4_pen.json          Active A4 pen Demo v1 profile
  mondrian_12x12_paint.json    Legacy/future 12-inch color profile

docs/
  Rob_Ross_Prototype_v1.md     Current first prototype direction
  Rob_Ross_Discuss.md          Early discussion and product brainstorming
  painting-paths-format.md     Format reference for output/painting_paths.json

Image_Process/
  README.md                    Overview of the image-processing module folders
  mondrian/                    Mondrian-style artwork/path generation pipeline
    README.md                   Detailed script and config workflow documentation
    config_loader.py            Shared JSON config loading and validation
    mondrian_generator.py       Generates SVG preview and painting_plan.json
    generate_painting_paths.py  Converts painting_plan.json into painting_paths.json
    generate_test_line.py       Generates the single 50 mm first-contact test line
    path_validation.py          Validates generated path command data
    tests/                      Unit tests for this pipeline (run: python3 -m unittest discover Image_Process/mondrian/tests)

ros2/
  robross_aubo.repos           vcstool manifest for the RobRoss Aubo driver fork
  robross_painter/             ROS 2 package that executes path files through MoveIt

output/
  mondrian_preview.svg         Human preview of generated artwork
  painting_plan.json           Intermediate artwork operations
  path_preview.svg             Human preview of generated stroke paths
  path_animation.svg           Animated stroke-order preview (open in a browser)
  painting_paths.json          Intermediate robot-style path commands
  test_line_paths.json         Single 50 mm test-line path file
  test_line_preview.svg        Human preview of the test line
```

---

## Key Concepts / 关键概念

### Painting plan

`painting_plan.json` describes the artwork at a higher level: rectangles, lines, canvas metadata, coordinate system, and operation order.

### Painting paths

`painting_paths.json` describes lower-level drawing commands that a future robot adapter can translate into physical robot motion.

### Coordinate system

All generated coordinates use millimeters.

```text
Origin: top-left of the paper/canvas
x direction: right
y direction: down
```

For Demo v1, valid A4 coordinates should stay within:

```text
0 <= x <= 210
0 <= y <= 297
```

In addition, the entire artwork (border included) is generated at least
`canvas.margin_mm` inside the paper edge (10 mm in the Demo v1 config),
so the pen never draws right at the physical paper edge. For Demo v1
this means all strokes actually fall within `10 <= x <= 200` and
`10 <= y <= 287`.

### Robot adapter

`ros2/robross_painter` converts canvas coordinates into robot poses and sends them through MoveIt using the Aubo i5 planning group. It is intentionally separate from the generator scripts: artwork files stay in canvas millimeters, while robot calibration and execution parameters live in the ROS 2 package config.

Robot calibration data, such as taught paper corners, safe Z height, contact Z height, tool center point, and home pose, should live in ROS/hardware config, not inside the artwork config.

---

## Team Roles / 团队分工

The project currently involves three core areas:

| Role | Focus |
| --- | --- |
| Project coordination + software contribution | Prototype scope, documentation, requirements, testing flow, path-generation support. |
| Software development | Script architecture, config workflow, path generation, validation, ROS 2 robot execution. |
| Hardware engineering | Aubo i5 setup, pen/tool mounting, paper/canvas stand, physical calibration, safety, and test reliability. |

The team should avoid building product ideas, software, and hardware separately. Every near-term decision should connect back to the first working prototype: a robot that can reliably draw a simple preprocessed artwork on A4 paper.

---

## Guidance for LLM Coding Agents / 给 LLM 代码助手的说明

When working in this repo:

- Treat Demo v1 A4 pen drawing as the active requirement.
- Do not treat early 12-inch color painting notes as current requirements unless explicitly asked.
- Use `configs/demo_v1_a4_pen.json` by default.
- Keep direct robot execution in `ros2/robross_painter`; do not put robot calibration poses inside artwork/path config files.
- Preserve the separation between artwork generation, path generation, validation, and robot execution.
- Prefer simple, readable Python using the standard library unless a dependency is clearly justified.
- Update Markdown when behavior or project decisions change.

Before making code changes, read:

```text
README.md
Image_Process/mondrian/README.md
docs/Rob_Ross_Prototype_v1.md
docs/painting-paths-format.md
```

---

## Current Next Steps / 当前下一步

Recommended technical next step:

> Build the ROS 2 workspace with the RobRoss Aubo fork, run the fake-hardware RViz flow, then test only after the Aubo i5 is physically calibrated to the paper.
