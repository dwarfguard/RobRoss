# Prototype v1: A4 Pen Demo / 第一版原型：A4 纸笔绘图测试

**Status:** Active prototype specification  
**Last updated:** 2026-07-07  
**Primary audience:** Human collaborators, software contributors, hardware contributors, and LLM coding agents  
**Current target:** Aubo i5 hardware test using A4 paper and a pen

---

## 1. Purpose / 目标

Prototype v1 is the first practical hardware-focused version of RobRoss.

The goal is **not** to build the full public art installation yet. The goal is to prove that the system can reliably convert a prepared artwork into drawing paths and eventually have the Aubo i5 robot arm execute those paths on a real physical surface.

第一版原型的目标不是直接完成最终商业展示产品，而是先验证：系统能否把预设图案转换成绘图路径，并让 Aubo i5 机械臂在真实纸面上稳定执行这些路径。

The guiding principle is:

> Reliability, simplicity, and visual clarity first. Artistic flexibility later.

---

## 2. Current Prototype Scope / 当前原型范围

| Area | Decision |
| --- | --- |
| Paper / Canvas | A4 portrait paper, 210 mm x 297 mm |
| Drawing tool | Pen on paper |
| Color | No color; monochrome black line drawing |
| Artwork type | Preset artwork only |
| First style | Mondrian-inspired geometric line drawing / sketch outline tracing |
| Path generation | Preprocessed, deterministic path instructions |
| Robot target | Aubo i5 robot arm |
| Robot execution | Future step; current repo outputs intermediate path files |

当前第一版原型使用 A4 纸、黑色笔、预设图案和预处理路径。重点是验证机械臂绘图流程，而不是完整绘画表现。

---

## 3. Non-Goals / 第一版暂不做

The following features may be valuable later, but they are **not required for Prototype v1**:

- No acrylic paint or oil paint.
- No live color mixing.
- No brush washing or paint dipping.
- No multiple brush/tool changes.
- No user-uploaded custom images.
- No real-time AI decision-making during robot execution.
- No real-time computer vision correction.
- No automatic mistake correction.
- No final commercial kiosk/user interface.
- No direct Aubo i5 motor-control code in the current path-generation scripts.

这些功能未来可能需要，但不属于当前第一版原型范围。第一版只需要证明最基本、最可靠的机器人绘图流程。

---

## 4. Success Criteria / 成功标准

Prototype v1 is successful if the team can demonstrate the following:

1. The software generates an A4-compatible artwork plan.
2. The generated path coordinates stay inside the A4 bounds:
   - `0 <= x <= 210`
   - `0 <= y <= 297`
3. The path preview visually matches the intended artwork.
4. The path data passes validation.
5. The team can map the A4 canvas coordinate system to the physical robot workspace.
6. The Aubo i5 can draw a small test line on paper with the expected:
   - position,
   - direction,
   - scale,
   - pen contact,
   - and safe motion.
7. The robot can eventually draw a simple Mondrian-style line composition without human correction during the run.

The minimum successful hardware demo is:

> The Aubo i5 draws a correctly oriented 50 mm test line on A4 paper using the same coordinate system as the generated path file.

---

## 5. Failure Criteria / 失败标准

Prototype v1 should be considered not ready for full demo execution if any of the following are true:

- The generated coordinates exceed A4 bounds.
- The SVG preview and JSON path data do not match.
- The robot coordinate frame does not match the paper coordinate frame.
- The pen moves in the wrong direction, mirrored direction, or wrong scale.
- The pen presses too hard, does not touch the paper, or tears the paper.
- The robot path cannot be safely dry-run above the paper.
- Emergency stop, workspace clearance, or operator supervision is not ready.
- The team cannot explain how `painting_paths.json` maps to physical robot movement.

---

## 6. Coordinate System / 坐标系统

All generated drawing paths use millimeters.

```text
Origin: top-left of the paper
x direction: right
y direction: down
```

For A4 portrait paper:

```text
width: 210 mm
height: 297 mm
```

Valid Demo v1 coordinate range:

```text
0 <= x <= 210
0 <= y <= 297
```

The artwork is additionally generated with a safety margin
(`canvas.margin_mm`, 10 mm in the Demo v1 config): every stroke,
including the border, stays at least that far inside the paper edge, so
the pen never draws right at the physical edge. For Demo v1 all strokes
therefore fall within `10 <= x <= 200` and `10 <= y <= 287`.

This coordinate system describes the paper/canvas, not the robot base coordinate system. A future robot adapter must transform paper coordinates into Aubo i5 robot poses.

---

## 7. Software Pipeline / 软件流程

The current repo pipeline is:

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

### Current active config

Use this profile for Prototype v1:

```text
configs/demo_v1_a4_pen.json
```

This config defines:

- A4 portrait paper.
- Monochrome line drawing.
- Pen-like path settings.
- Output file names.

### Legacy/future config

This profile preserves the older 12-inch color-paint behavior:

```text
configs/mondrian_12x12_paint.json
```

Do not use the 12-inch color profile for the first Aubo i5 pen-on-paper test unless the team explicitly decides to test legacy/future behavior.

---

## 8. How to Generate Demo v1 Files / 如何生成第一版文件

From the repo root, run:

```bash
python3 scripts/mondrian_generator.py --config configs/demo_v1_a4_pen.json --seed 123
```

Then run:

```bash
python3 scripts/generate_painting_paths.py --config configs/demo_v1_a4_pen.json
```

Expected outputs:

```text
output/mondrian_preview.svg
output/painting_plan.json
output/path_preview.svg
output/painting_paths.json
```

Review the SVG previews before using the JSON path data.

Always use the same config for both scripts. Do not generate the plan with one config and the paths with another config.

---

## 9. Meaning of Generated Files / 输出文件含义

### `output/mondrian_preview.svg`

Human-readable visual preview of the generated artwork.

Use this to check whether the artwork looks reasonable before generating or testing paths.

### `output/painting_plan.json`

Intermediate artwork plan.

This file describes the artwork in higher-level operations such as lines and, in color modes, rectangles. It is not robot motor code.

### `output/path_preview.svg`

Human-readable preview of the actual generated stroke paths.

Use this to check stroke order, direction, and whether the robot-style paths match the intended drawing.

### `output/painting_paths.json`

Intermediate robot-style path command file.

This file contains abstract commands such as:

```text
select_tool
dip_paint
move_to
lower_tool
paint_stroke
lift_tool
```

For Demo v1, `select_tool` and `dip_paint` may be treated as no-op/log-only commands because the test uses a single pen and no paint.

Important:

> `painting_paths.json` is not direct Aubo i5 motor code.

A future adapter must convert these canvas-space commands into real robot poses and motion commands.

---

## 10. Hardware Assumptions / 硬件假设

Prototype v1 assumes:

- Aubo i5 robot arm is available for hardware testing.
- A pen can be mounted securely as the drawing tool.
- A4 paper can be fixed flat in the robot workspace.
- The robot can safely move above the paper before touching it.
- The team can define:
  - safe Z height,
  - travel Z height,
  - contact Z height,
  - paper top-left point,
  - paper top-right point,
  - paper bottom-left point,
  - and tool center point / pen tip position.

Robot calibration should be stored separately from artwork configs. A future hardware config may look like:

```text
configs/aubo_i5_lab_setup.json
```

Do not put robot base poses, Z heights, or calibration data inside the artwork/path config files.

---

## 11. First Hardware Test Procedure / 第一次硬件测试流程

The first hardware test should be gradual. Do not begin with a full drawing.

### Step 1: Software-only review

- Generate Demo v1 files.
- Open `output/mondrian_preview.svg`.
- Open `output/path_preview.svg`.
- Check that `painting_paths.json` validation passes.
- Confirm all coordinates fit inside A4 bounds.

### Step 2: Robot workspace setup

- Secure the A4 paper.
- Mount the pen.
- Confirm the emergency stop is accessible.
- Clear the robot workspace.
- Confirm the operator can stop the robot immediately.

### Step 3: Paper coordinate calibration

Teach or record these physical points:

```text
paper_top_left
paper_top_right
paper_bottom_left
```

Use these points to map generated paper coordinates into robot coordinates.

### Step 4: Dry run above paper

- Keep the pen lifted above the paper.
- Move to the four A4 corners at safe height.
- Confirm the robot moves in the expected orientation:
  - increasing x moves right,
  - increasing y moves down.

### Step 5: Single-line contact test

Draw one simple test line before running any generated artwork.

Recommended first line:

```text
Start: (80, 140)
End:   (130, 140)
Length: 50 mm
```

Generate this line as a real path file with:

```bash
python3 scripts/generate_test_line.py
```

so the first stroke uses the same `painting_paths.json` format (and
future robot adapter) as the full artwork. Output:
`output/test_line_paths.json` and `output/test_line_preview.svg`.

Check:

- Is the line horizontal?
- Is the line approximately 50 mm?
- Did the pen touch correctly?
- Was the pressure too high or too low?
- Did the robot move safely?

### Step 6: Small path subset

Run only a small number of generated strokes.

Do not run the full path until the team confirms orientation, scale, contact height, and safety.

### Step 7: Full Demo v1 path

Run the full generated A4 path only after the previous steps pass.

---

## 12. Safety Notes / 安全注意事项

Before any robot movement:

- Confirm emergency stop access.
- Confirm no one is inside the robot motion area.
- Confirm paper and tool mount are secure.
- Confirm the robot speed is appropriate for testing.
- Start with slow motion.
- Start with dry-run movement above the paper.
- Do not test paint or liquid materials during Demo v1.
- Do not leave the robot running unattended.

The first prototype should prioritize safety and reliability over visual polish.

---

## 13. Team Responsibilities / 团队职责

| Area | Responsibility |
| --- | --- |
| Project coordination | Keep the prototype scope clear, document decisions, track risks and next steps. |
| Software | Generate artwork plans, generate path commands, validate path data, prepare future robot adapter requirements. |
| Hardware | Prepare Aubo i5, pen mount, paper fixture, robot calibration, safety setup, and physical testing. |
| Testing | Compare expected vs. actual robot behavior and record issues clearly. |

All team members should use this document as the active Prototype v1 source of truth.

---

## 14. Guidance for LLM Coding Agents / 给 LLM 代码助手的说明

When working on this repo:

- Treat this file as the active Prototype v1 requirement document.
- Use `configs/demo_v1_a4_pen.json` by default.
- Do not assume the current prototype uses a 12-inch canvas.
- Do not assume the current prototype uses color or paint.
- Do not add Aubo SDK integration unless explicitly requested.
- Do not place robot calibration data inside artwork configs.
- Preserve the separation between:
  - artwork generation,
  - path generation,
  - path validation,
  - and future robot execution.
- Keep code simple and readable.
- Use Python standard library unless a dependency is clearly necessary.
- Update Markdown when requirements or behavior change.

Before changing code, read:

```text
README.md
docs/Rob_Ross_Prototype_v1.md
docs/painting-paths-format.md
scripts/README.md
configs/demo_v1_a4_pen.json
```

---

## 15. Open Questions / 未解决问题

The following questions still need hardware testing or team decisions:

1. What pen type works best with the Aubo i5 end effector?
2. What is the safest and most reliable pen mounting method?
3. What contact Z height gives consistent lines without damaging paper?
4. What robot speed is safe and visually clean for pen strokes?
5. Should Demo v1 draw only grid lines, or include simple outline tracing later?
6. Should the generated first hardware test use a very small fixed path instead of a full Mondrian composition?
7. What file or script should eventually translate `painting_paths.json` into Aubo i5 movement commands?
8. What format should the future robot calibration config use?

---

## 16. Current Next Step / 当前下一步

The next practical milestone is:

> Generate an A4 line-only path, review the SVG previews, calibrate the Aubo i5 to the paper, and draw one correctly oriented 50 mm test line.

Once that succeeds, the team can move toward running a small subset of the generated Mondrian paths.
