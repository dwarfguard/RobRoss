# Hardware Test Checklist: Aubo i5 A4 Pen Demo

**Status:** Active hardware-test checklist  
**Prototype:** Demo v1 — A4 pen-on-paper drawing  
**Robot target:** Aubo i5  
**Purpose:** Guide the first safe hardware test using generated RobRoss path files.

---

## 1. Test Goal

The first hardware goal is not to draw a full artwork.

The first goal is:

> Draw one correctly oriented 50 mm line on A4 paper.

Only after that succeeds should the team test a small subset of generated paths, then eventually a full Demo v1 Mondrian-style path.

---

## 2. Required Files

Before hardware testing, generate and review:

```text
output/mondrian_preview.svg
output/painting_plan.json
output/path_preview.svg
output/path_animation.svg
output/painting_paths.json
```

`path_animation.svg` shows the strokes drawing in execution order (open
in a web browser) — a good way to sanity-check stroke order and travel
moves before moving the robot.

Use the Demo v1 config:

```text
configs/demo_v1_a4_pen.json
```

Recommended generation commands:

```bash
python3 Image_Process/mondrian/mondrian_generator.py --config configs/demo_v1_a4_pen.json --seed 123
python3 Image_Process/mondrian/generate_painting_paths.py --config configs/demo_v1_a4_pen.json
```

---

## 3. Software Pre-Check

Before moving the robot, confirm:

- [ ] `output/mondrian_preview.svg` looks correct.
- [ ] `output/path_preview.svg` looks correct.
- [ ] `output/painting_paths.json` exists.
- [ ] `painting_paths.json` validation passes.
- [ ] Canvas is A4 portrait:
  - [ ] width = `210 mm`
  - [ ] height = `297 mm`
- [ ] Coordinates stay inside A4 bounds:
  - [ ] `0 <= x <= 210`
  - [ ] `0 <= y <= 297`
- [ ] The path file is understood as intermediate path data, not direct Aubo motor code.

Do not begin hardware testing if the generated files fail these checks.

---

## 4. Workspace Safety Check

Before powering or moving the robot:

- [ ] Emergency stop is visible and reachable.
- [ ] Operator knows how to stop the robot immediately.
- [ ] Robot workspace is clear of people and loose objects.
- [ ] A4 paper is secured flat and cannot slide.
- [ ] Pen is securely mounted.
- [ ] Pen tip is not broken or loose.
- [ ] No paint, liquid, or open containers are present for Demo v1.
- [ ] Cables are clear of robot motion.
- [ ] Robot speed is set appropriately for early testing.
- [ ] A human operator is present for the entire test.

Do not leave the robot running unattended.

---

## 5. Physical Setup

Prepare the paper and tool:

- [ ] Place A4 paper in portrait orientation.
- [ ] Secure paper with tape, clips, or a flat fixture.
- [ ] Mount pen to the end effector.
- [ ] Confirm the pen cannot rotate or shift during motion.
- [ ] Confirm the pen tip is the intended drawing point.
- [ ] Confirm the robot can move above the full A4 area without collision.

Recommended paper coordinate convention:

```text
top-left     = (0, 0)
top-right    = (210, 0)
bottom-left  = (0, 297)
bottom-right = (210, 297)
```

---

## 6. Calibration Points

Record or teach these three paper points:

```text
paper_top_left
paper_top_right
paper_bottom_left
```

These points define the mapping from generated canvas coordinates to physical robot coordinates.

Checklist:

- [ ] `paper_top_left` is recorded.
- [ ] `paper_top_right` is recorded.
- [ ] `paper_bottom_left` is recorded.
- [ ] The direction from top-left to top-right matches positive X.
- [ ] The direction from top-left to bottom-left matches positive Y.
- [ ] The measured distance from top-left to top-right is approximately 210 mm.
- [ ] The measured distance from top-left to bottom-left is approximately 297 mm.

Do not continue if the physical coordinate frame is rotated, mirrored, or scaled incorrectly.

---

## 7. Z-Height Setup

Define and record:

```text
safe_z
travel_z
contact_z
```

Meaning:

| Value | Meaning |
| --- | --- |
| `safe_z` | Height clearly above the paper and fixtures. |
| `travel_z` | Normal movement height above paper between strokes. |
| `contact_z` | Pen contact height for drawing. |

Checklist:

- [ ] `safe_z` clears paper, clips, tape, and fixture.
- [ ] `travel_z` clears the paper during non-drawing moves.
- [ ] `contact_z` touches paper lightly.
- [ ] Pen contact does not tear paper.
- [ ] Pen contact leaves a visible line.
- [ ] Z-height values are recorded for repeatability.

---

## 8. Dry Run: No Paper Contact

Before drawing, run motion above the paper.

Checklist:

- [ ] Pen is lifted above paper.
- [ ] Robot moves to near top-left safely.
- [ ] Robot moves to near top-right safely.
- [ ] Robot moves to near bottom-left safely.
- [ ] Robot moves to near bottom-right safely.
- [ ] Motion direction matches expected paper coordinates.
- [ ] No collision occurs.
- [ ] Operator can stop the robot immediately if needed.

Expected direction:

```text
Increasing x moves right.
Increasing y moves down.
```

---

## 9. First Contact Test: 50 mm Line

Draw only one simple line first.

Recommended test line in paper coordinates:

```text
Start: (80, 140)
End:   (130, 140)
Length: 50 mm
```

Generate this line as a real path file (same format as the full artwork,
so the robot adapter is exercised the same way):

```bash
python3 Image_Process/mondrian/generate_test_line.py
```

This writes `output/test_line_paths.json` and
`output/test_line_preview.svg`, validated like any other path file.

Checklist:

- [ ] Robot moves to start point at travel height.
- [ ] Robot lowers to contact height.
- [ ] Robot draws from `(80, 140)` to `(130, 140)`.
- [ ] Robot lifts after drawing.
- [ ] Line is horizontal.
- [ ] Line is approximately 50 mm long.
- [ ] Line direction is correct.
- [ ] Pen pressure is acceptable.
- [ ] Paper is not damaged.
- [ ] Robot motion remains safe.

If this fails, do not run generated artwork paths yet.

---

## 10. Curve Test Card

After the 50 mm test line succeeds, generate the fixed curve test card:

```bash
python3 Image_Process/mondrian/generate_curve_test.py
```

This writes `output/curve_test_paths.json` and
`output/curve_test_preview.svg`. It tests a smooth S-curve, a closed circle, a
sine squiggle, and sharp right-angle and acute corners as four separate
continuous paths.

Checklist:

- [ ] `curve_test_paths.json` validation passes without warnings.
- [ ] Compare the paths against `curve_test_preview.svg`.
- [ ] Confirm all commands stay inside A4 bounds.
- [ ] Run dry above paper first.
- [ ] Run with pen contact only after dry run passes.
- [ ] Confirm the S-curve and squiggle are smooth.
- [ ] Confirm the circle closes without a visible gap or overshoot.
- [ ] Confirm right-angle and acute corners remain distinct.
- [ ] Confirm the tool lifts between all four shapes.
- [ ] Confirm line quality and pen pressure remain uniform.

Do not run the full generated path until the curve test card succeeds.

---

## 11. Full Demo v1 Path Test

Run the full path only after:

- [ ] Software pre-check passed.
- [ ] Workspace safety check passed.
- [ ] Calibration points are correct.
- [ ] Z heights are correct.
- [ ] Dry run passed.
- [ ] 50 mm line test passed.
- [ ] Curve test card passed.
- [ ] Operator and emergency stop are ready.

During the full run:

- [ ] Watch for drift.
- [ ] Watch for pen slipping.
- [ ] Watch for unexpected Z behavior.
- [ ] Watch for paper movement.
- [ ] Stop immediately if motion becomes unsafe.

---

## 12. Test Result Notes

Record the result after each hardware session.

```text
Date:
Operator:
Robot:
Tool / pen:
Paper mounting method:
Config used:
Seed used:
Generated files checked:
Calibration method:
safe_z:
travel_z:
contact_z:

50 mm line result:
- Position:
- Direction:
- Length:
- Pen pressure:
- Issues:

Curve test card result:
- Shapes completed:
- Issues:

Full path result:
- Completed? yes/no
- Issues:

Next changes needed:
```

---

## 13. Stop Conditions

Stop testing immediately if:

- The robot moves in an unexpected direction.
- The robot moves outside the intended paper area.
- The pen digs into or tears the paper.
- The paper shifts.
- The end effector becomes loose.
- Any person enters the robot workspace.
- Emergency stop access is blocked.
- The operator is unsure what the robot will do next.

Safety is more important than completing the drawing.

---

## 14. After-Test Actions

After each test:

- [ ] Save notes.
- [ ] Photograph or scan the drawn output.
- [ ] Compare physical output to `path_preview.svg`.
- [ ] Record calibration problems.
- [ ] Record software/path issues separately from hardware issues.
- [ ] Decide the next smallest test to run.
- [ ] Update documentation if the process changes.

Do not jump from a failed test directly to a larger test.
