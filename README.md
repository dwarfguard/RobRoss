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
| Robot code | Not implemented yet; current outputs are intermediate path files |

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
scripts/mondrian_generator.py
  ↓
output/painting_plan.json
  ↓
scripts/generate_painting_paths.py
  ↓
output/painting_paths.json
  ↓
Future Aubo i5 adapter
  ↓
Robot motion
```

Important: `painting_paths.json` is **not** direct robot motor code. It is an intermediate representation using millimeter coordinates and abstract commands such as `move_to`, `lower_tool`, `paint_stroke`, and `lift_tool`.

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
python3 scripts/mondrian_generator.py --config configs/demo_v1_a4_pen.json --seed 123
```

Generate path commands from that plan:

```bash
python3 scripts/generate_painting_paths.py --config configs/demo_v1_a4_pen.json
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
python3 scripts/generate_test_line.py
```

Run the unit tests:

```bash
python3 -m unittest discover tests
```

For the legacy 12-inch colored profile:

```bash
python3 scripts/mondrian_generator.py --config configs/mondrian_12x12_paint.json --seed 123
python3 scripts/generate_painting_paths.py --config configs/mondrian_12x12_paint.json
```

Always run both scripts with the same config profile. Mixing profiles can produce confusing or invalid results.

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

scripts/
  README.md                    Detailed script and config workflow documentation
  config_loader.py             Shared JSON config loading and validation
  mondrian_generator.py        Generates SVG preview and painting_plan.json
  generate_painting_paths.py   Converts painting_plan.json into painting_paths.json
  generate_test_line.py        Generates the single 50 mm first-contact test line
  path_validation.py           Validates generated path command data

tests/
  test_*.py                    Unit tests (run: python3 -m unittest discover tests)

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

The future Aubo i5 adapter should convert canvas coordinates into robot poses. That adapter is not part of the current generator/path scripts yet.

Robot calibration data, such as taught paper corners, safe Z height, contact Z height, tool center point, and home pose, should live in a separate future hardware config, not inside the artwork config.

---

## Team Roles / 团队分工

The project currently involves three core areas:

| Role | Focus |
| --- | --- |
| Project coordination + software contribution | Prototype scope, documentation, requirements, testing flow, path-generation support. |
| Software development | Script architecture, config workflow, path generation, validation, future robot-control integration. |
| Hardware engineering | Aubo i5 setup, pen/tool mounting, paper/canvas stand, physical calibration, safety, and test reliability. |

The team should avoid building product ideas, software, and hardware separately. Every near-term decision should connect back to the first working prototype: a robot that can reliably draw a simple preprocessed artwork on A4 paper.

---

## Guidance for LLM Coding Agents / 给 LLM 代码助手的说明

When working in this repo:

- Treat Demo v1 A4 pen drawing as the active requirement.
- Do not treat early 12-inch color painting notes as current requirements unless explicitly asked.
- Use `configs/demo_v1_a4_pen.json` by default.
- Do not add Aubo SDK integration unless explicitly requested.
- Do not place robot calibration poses inside artwork/path config files.
- Preserve the separation between artwork generation, path generation, validation, and future robot execution.
- Prefer simple, readable Python using the standard library unless a dependency is clearly justified.
- Update Markdown when behavior or project decisions change.

Before making code changes, read:

```text
README.md
scripts/README.md
docs/Rob_Ross_Prototype_v1.md
docs/painting-paths-format.md
```

---

## Current Next Steps / 当前下一步

Recommended documentation cleanup:

1. Keep this root README focused on the current prototype and repo navigation.
2. Expand `docs/Rob_Ross_Prototype_v1.md` into a complete active prototype spec.
3. Add an `AGENTS.md` file for LLM/coding-agent instructions.
4. Add a hardware test checklist for the first Aubo i5 pen-on-paper session.
5. Archive or clearly label older brainstorming so it is not confused with active requirements.

Recommended technical next step:

> Generate a small A4 line-only path, review the SVG previews, then test only after the Aubo i5 is physically calibrated to the paper.
