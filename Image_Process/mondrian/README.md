# Mondrian Artwork Pipeline

This module generates a Mondrian-style layout, converts it into continuous
drawing paths, validates those paths, and creates static and animated SVG
previews. Run all commands from the repository root.

## Quick Start

Generate a reproducible Demo v1 artwork and its paths:

```bash
python3 Image_Process/mondrian/mondrian_generator.py \
  --config configs/demo_v1_a4_pen.json \
  --seed 123
python3 Image_Process/mondrian/generate_painting_paths.py \
  --config configs/demo_v1_a4_pen.json
```

Review these files before robot execution:

```text
output/mondrian_preview.svg    Artwork preview
output/path_preview.svg        Static execution-path preview
output/path_animation.svg      Animated execution order
output/painting_paths.json     Validated path commands
```

Open `path_animation.svg` in a browser and reload it to replay. Always use the
same config for both generation commands; the path generator reads the plan
created by the artwork generator.

Run the tests with:

```bash
python3 -m unittest discover Image_Process/mondrian/tests
```

## Profiles

| Config | Purpose |
| --- | --- |
| `configs/demo_v1_a4_pen.json` | Active profile: A4 portrait paper, 10 mm margin, monochrome 1 mm pen lines. |
| `configs/mondrian_12x12_paint.json` | Legacy profile: 12-inch square canvas, colored blocks, and brush-style fills. |

Both generator scripts default to the Demo v1 profile when `--config` is
omitted. Use a fixed `--seed` to reproduce an artwork; without one, the
generator prints the random seed used.

## Pipeline

```text
JSON profile
    |
    v
mondrian_generator.py
    +-- painting_plan.json
    +-- mondrian_preview.svg
    |
    v
generate_painting_paths.py
    +-- painting_paths.json
    +-- path_preview.svg
    +-- path_animation.svg
```

`mondrian_generator.py` recursively subdivides the configured canvas. In
monochrome mode it emits grid and border lines only; in color mode it can also
emit filled rectangles. The SVG and JSON plan are built from the same layout.

`generate_painting_paths.py` converts plan operations into polylines, then:

- connects touching lines into continuous paths;
- fills rectangles with serpentine paths;
- orders paths with greedy nearest-neighbor travel;
- emits `move_to`, `lower_tool`, `paint_path`, and `lift_tool` sequences;
- renders static and animated previews from the same command list.

The four border lines therefore become one closed path rather than four
separate strokes. `paint_stroke` remains supported for simple straight paths,
including the first-contact test line.

## Configuration

Profiles describe artwork generation only. Robot pose, tool geometry, and
motion limits belong in `ros2/robross_painter/config/`.

| Field | Meaning |
| --- | --- |
| `canvas.width_mm`, `canvas.height_mm` | Physical canvas dimensions. |
| `canvas.margin_mm` | Minimum distance between the complete artwork and the canvas edge. |
| `canvas.origin` | Must currently be `top-left`. |
| `artwork.palette_mode` | `monochrome` for lines only or `color` for accent fills. |
| `artwork.min_cell_fraction` | Minimum cell size used during subdivision. |
| `artwork.min_split_depth`, `artwork.max_split_depth` | Subdivision depth limits. |
| `path_generation.tool_width_mm` | Width of one pen or paint stroke. |
| `path_generation.stroke_overlap_ratio` | Overlap between adjacent fill rows, from 0 inclusive to 1 exclusive. |
| `path_generation.edge_inset_mm` | Distance between fill strokes and a region's edge. |
| `output.*` | Output directory and generated filenames. |

`config_loader.py` validates required sections, types, canvas dimensions,
margins, subdivision limits, path settings, and output names before generation.
Invalid profiles fail with an itemized error message.

## Test Line

Create the standard 50 mm first-contact path:

```bash
python3 Image_Process/mondrian/generate_test_line.py
```

Use custom endpoints when needed:

```bash
python3 Image_Process/mondrian/generate_test_line.py \
  --start 80 140 \
  --end 130 140
```

This writes `output/test_line_paths.json` and
`output/test_line_preview.svg` without replacing the full artwork outputs.
Endpoints are in canvas millimeters.

## Curve Test Card

After the 50 mm first-contact line succeeds, generate the deterministic curve
test card:

```bash
python3 Image_Process/mondrian/generate_curve_test.py
```

This writes `output/curve_test_paths.json` and
`output/curve_test_preview.svg`. The card contains four independently lifted
`paint_path` trajectories: a smooth S-curve, a closed circle, a sine squiggle,
and a sparse polyline with right-angle and acute corners. The geometry is
hardcoded so repeated hardware runs can be compared against the same reference.

Do not use this card for first contact. Dry-run it first and run it on paper
only after `output/test_line_paths.json` passes the hardware gate.

## Validation

All path generators call `path_validation.validate_painting_paths()` before
exiting. The result is stored in `painting_paths.json`:

```json
{
  "validation": {
    "passed": true,
    "errors": [],
    "warnings": []
  }
}
```

Validation errors produce a nonzero exit code. Generated files are still
written so failures can be inspected. The Python validator reports unknown
commands as warnings for forward-compatible tooling, but the robot executor
accepts only its explicitly supported command set and rejects unknown commands.

See [painting_paths.json format](../../docs/painting-paths-format.md) for the
coordinate system, command fields, and validation rules.

## Robot Handoff

`painting_paths.json` contains canvas-space instructions, not Aubo motor
commands. Before execution:

1. Confirm generation completed without validation errors.
2. Inspect both path previews for unexpected travel or ordering.
3. Follow the [robross_painter guide](../../ros2/robross_painter/README.md) for RViz.
4. For a real arm, complete the [hardware preflight](../../ros2/robross_painter/PREFLIGHT.md) in order.

Do not place robot calibration data in an artwork profile.
