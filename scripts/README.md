# Scripts

Utility scripts for this repo. Each script gets its own section below —
when adding a new script, copy the section format used for
`mondrian_generator.py`.

---

## Config-driven workflow

`mondrian_generator.py` and `generate_painting_paths.py` are both driven
by a single JSON config file (in `configs/`) instead of hardcoded
constants. A config fully describes one "profile": canvas size, artwork
style/palette, path/tool settings, and output file names. This is what
lets the same pipeline target a 12in square paint canvas today and an A4
pen-and-paper canvas for Demo v1 without forking the scripts.

Both scripts take the same flag:

```bash
python3 scripts/mondrian_generator.py --config configs/<profile>.json --seed 123
python3 scripts/generate_painting_paths.py --config configs/<profile>.json
```

`--config` defaults to `configs/demo_v1_a4_pen.json` if omitted. Always
pass the *same* `--config` to both scripts in a run — `generate_painting_paths.py`
reads `output/<painting_plan_file>` written by `mondrian_generator.py`
using that config's `output.directory`/`output.painting_plan_file`, so a
mismatched config will either fail to find the plan or silently mix
settings from two different profiles.

### Available profiles

| Config | Canvas | Palette | Tool | Use case |
| --- | --- | --- | --- | --- |
| `configs/demo_v1_a4_pen.json` | 210mm x 297mm (A4 portrait), 10mm safety margin | monochrome (black lines only, no fills) | 1mm pen, 0% overlap, 5mm edge inset | Demo v1: first Aubo i5 hardware test — pen on paper. |
| `configs/mondrian_12x12_paint.json` | 304.8mm x 304.8mm (12in square), no margin | classic red/yellow/blue (+occasional gray) | 10mm brush, 25% overlap, 3mm edge inset | Original colored Mondrian behavior, preserved for backward compatibility. |

### Important config fields

| Field | Meaning |
| --- | --- |
| `canvas.width_mm` / `canvas.height_mm` | Canvas size. Not assumed square — width and height are always handled separately (including border generation). |
| `canvas.margin_mm` | Safety margin: the entire artwork, border included, stays this far inside the physical canvas/paper edge, so the tool never draws right at the edge. Optional, defaults to `0`. Must satisfy `2 * margin < min(width, height)`. Demo v1 uses `10.0`. |
| `canvas.origin` | Must currently be `"top-left"` (the only supported origin). |
| `artwork.palette_mode` | `"monochrome"` skips all colored rectangle fills — only grid lines + border are generated (see [Demo v1 monochrome behavior](#demo-v1-monochrome-behavior)). `"color"` picks accent-colored blocks like the original generator. |
| `artwork.accent_colors` / `artwork.neutral_accent_color` | Palette used in `"color"` mode. Ignored in `"monochrome"` mode. |
| `artwork.min_cell_fraction` / `artwork.max_split_depth` | Recursive subdivision tuning — same meaning as the old `MIN_CELL_FRACTION`/`MAX_SPLIT_DEPTH` constants. |
| `artwork.min_split_depth` | Depths below this always subdivide (when the cell is big enough), guaranteeing the artwork is never an empty border-only rectangle. Optional integer, defaults to `1`. Both current profiles use `2`. |
| `path_generation.tool_width_mm` | Width of a single paint/pen stroke. |
| `path_generation.stroke_overlap_ratio` | How much each fill stripe overlaps the previous one (`0 <= ratio < 1`). |
| `path_generation.edge_inset_mm` | How far inside a rectangle's edge strokes are kept. |
| `output.directory` | Where all generated files go (`output/` for both current profiles). |
| `output.painting_plan_file`, `.painting_paths_file`, `.preview_svg_file`, `.path_preview_svg_file` | Output file names within `output.directory`. |
| `output.path_animation_svg_file` | Animated path preview file name. Optional — defaults to `path_animation.svg` so configs without it keep working. |

Config files are loaded and validated by `config_loader.py` (see below) —
both scripts fail with a clear, itemized error message if a config is
missing required fields or has invalid values (negative canvas size,
`stroke_overlap_ratio >= 1`, wrong `origin`, etc.).

### Recommended first hardware flow (Demo v1)

1. Generate the A4 plan: `python3 scripts/mondrian_generator.py --config configs/demo_v1_a4_pen.json --seed <N>`.
2. Generate the A4 paths: `python3 scripts/generate_painting_paths.py --config configs/demo_v1_a4_pen.json`.
3. Review both SVG previews (`output/mondrian_preview.svg`, `output/path_preview.svg`) and check `output/painting_paths.json["validation"]["passed"]` is `true`.
4. Only feed `painting_paths.json` to the robot **after** the Aubo i5 has been calibrated to the physical paper/canvas position. Robot calibration (arm poses, home position, tool offsets) is intentionally out of scope for these configs — it belongs in a separate future config, e.g. `configs/aubo_i5_lab_setup.json`.

`painting_paths.json` is still an intermediate representation (mm
coordinates + abstract tool commands), not direct Aubo i5 robot motor
code — a separate translation step will turn this into actual robot
motion once hardware integration starts.

---

## config_loader.py

Shared config loading + validation used by both `mondrian_generator.py`
and `generate_painting_paths.py`, so they can't drift apart on what
counts as a valid config.

### Usage

```python
from config_loader import load_config, ConfigError

config = load_config("configs/demo_v1_a4_pen.json")
```

`load_config()` raises `ConfigError` (a `ValueError` subclass) if the
file is missing, isn't valid JSON, or fails validation — the message
lists every problem found, not just the first one.

### Functions

| Function | Purpose |
| --- | --- |
| `load_config(path)` | Reads and fully validates a config file; returns the parsed dict or raises `ConfigError`. |
| `validate_config(config)` | Returns a list of human-readable error strings (empty if valid). Used internally by `load_config()`. |

### Validation rules

- Required top-level sections: `canvas`, `artwork`, `path_generation`, `output`.
- `canvas.width_mm` / `canvas.height_mm` must be positive numbers.
- `canvas.origin` must be `"top-left"`.
- `path_generation.tool_width_mm` must be `> 0`.
- `path_generation.edge_inset_mm` must be `>= 0`.
- `path_generation.stroke_overlap_ratio` must satisfy `0 <= ratio < 1`.
- `output` must have non-empty `painting_plan_file`, `painting_paths_file`, `preview_svg_file`, `path_preview_svg_file`.

---

## mondrian_generator.py

Generates a randomized Mondrian/De Stijl-style layout sized and styled
by a config file (see [Config-driven workflow](#config-driven-workflow)):
a canvas recursively subdivided into cells, styled according to
`artwork.palette_mode` (colored accent blocks, or monochrome line-only),
separated by grid lines.

One run produces two outputs from the *same* in-memory layout:

- `output/mondrian_preview.svg` — human preview image.
- `output/painting_plan.json` — robot-friendly painting plan (ordered
  paint operations in millimeters, top-left origin, x right / y down).

Because both files are built from the same generated rectangles and
lines, the SVG and the JSON always describe the identical artwork.

### Usage

```bash
# Demo v1 (A4, pen, monochrome) — new layout every run
python3 scripts/mondrian_generator.py

# Reproduce a specific graphic
python3 scripts/mondrian_generator.py --config configs/demo_v1_a4_pen.json --seed 123

# Old 12in square colored profile
python3 scripts/mondrian_generator.py --config configs/mondrian_12x12_paint.json --seed 123
```

Output is written to `output/<preview_svg_file>` and
`output/<painting_plan_file>` (directory is created if missing; both
names come from the config's `output` section).

Every run prints the seed used and confirms each file, e.g.:

```
Generated output/mondrian_preview.svg (seed=2124073818)
Generated output/painting_plan.json (seed=2124073818)
```

Copy that seed value into `--seed` to regenerate the exact same graphic
and plan later — with the same config and seed, output is deterministic.

### Options

| Flag | Description |
| --- | --- |
| `--config PATH` | Path to a pipeline config JSON file. Defaults to `configs/demo_v1_a4_pen.json`. |
| `--seed N` | Use a fixed integer seed for reproducible output. Omit for a random seed (printed after generation). |

### How it works

1. `subdivide()` recursively splits the canvas rectangle into two
   (vertical or horizontal cut, random position) until cells hit a
   minimum size on their axis (`min_w`/`min_h`, derived from
   `artwork.min_cell_fraction`) or a depth-scaled stop probability
   triggers. This produces the leaf cells and the internal grid lines in
   one pass, and works for non-square canvases since each axis has its
   own minimum.
2. `generate_mondrian_layout()` reads canvas size and artwork settings
   from the config. In `"color"` mode it picks 2-4 leaf cells at random
   to fill with accent colors (rest stay `background_color`); in
   `"monochrome"` mode it skips accent-cell selection entirely (see
   [Demo v1 monochrome behavior](#demo-v1-monochrome-behavior)). Either
   way it picks a random stroke width (from `stroke_width_min_mm`/`_max_mm`)
   for the grid/border lines, labels every rectangle/line (e.g.
   `red_block_1`, `grid_line_3`, `border_top`), and returns the
   `Rect`/`Line` lists — the single source of truth for both outputs.
3. `render_svg()` turns those rectangles and lines into an SVG string
   sized to `canvas.width_mm`/`height_mm` (both the `width`/`height`
   attributes and the `viewBox`).
4. `build_painting_plan()` turns the same rectangles and lines into the
   JSON painting plan: only rectangles that aren't `background_color`
   become `paint_rectangle` operations, followed by all lines as
   `paint_line` operations (grid lines painted last so they clean up
   rectangle edges), plus canvas/coordinate metadata, a `config` block
   (profile name + config file path), and a `debug` summary.
5. `main()` loads and validates the config, resolves/reports the seed,
   builds the layout once, and writes both files from it.

### Demo v1 monochrome behavior

When `artwork.palette_mode == "monochrome"` (as in
`configs/demo_v1_a4_pen.json`), `generate_mondrian_layout()` never
samples accent cells or builds a color palette — no `paint_rectangle`
operations are produced at all, only `paint_line` (grid lines + border).
This matches the Demo v1 target: a pen tracing black lines on white
paper, no color fills, for the first Aubo i5 hardware test. Switch
`palette_mode` to `"color"` (and provide `accent_colors`) to get the
original colored-block behavior, as `configs/mondrian_12x12_paint.json` does.

### Config fields used

See [Important config fields](#important-config-fields) above for the
full list. `artwork.background_color`, `artwork.line_color`,
`artwork.stroke_width_min_mm`/`_max_mm`, and (color mode only)
`artwork.neutral_accent_probability` are also read here.

### Painting plan JSON

`painting_plan.json` (see `build_painting_plan()`) includes:

- `config` — `profile_name` and `source_file` (the config path used).
- `canvas` — size in mm and inches, origin corner.
- `coordinate_system` — x/y axis directions.
- `assumptions` — plain-language notes about paint order and canvas
  starting state, for anyone (human or downstream code) consuming the
  plan.
- `operations` — ordered list of `paint_rectangle` (colored blocks,
  solid fill; empty in monochrome mode) then `paint_line` (grid lines,
  then outer border) steps, each with a `label` for debugging.
- `debug` — seed used, rectangle/line/operation counts, and the list of
  colors used.

This is an intermediate plan, not robot motor code — it doesn't cover
things like brush lift/lower, travel paths between operations, or
paint mixing.

### Ideas for future features

- A `configs/aubo_i5_lab_setup.json`-style calibration config (robot
  poses, home position, tool offsets) — kept deliberately separate from
  the artwork/canvas/path configs here.
- Export to PNG/PDF alongside SVG.
- Batch mode: generate N variations of one config at once into `output/`.
- Real fill strategies (e.g. `horizontal_stripes`) instead of the
  current `solid_fill` placeholder, once the robot supports them.

---

## generate_painting_paths.py

Converts the abstract paint operations in `painting_plan.json` into
concrete robot-style stroke path commands (horizontal stripe fills for
rectangles, single strokes for lines), using tool settings from the
same config passed to `mondrian_generator.py`. This script is read-only
with respect to the Mondrian layout: it never generates a new random
layout, it only reads the plan that `mondrian_generator.py` already
produced.

One run produces three outputs from the *same* command list:

- `output/painting_paths.json` — ordered stroke commands for a robot to
  follow.
- `output/path_preview.svg` — static visual preview of those stroke paths.
- `output/path_animation.svg` — animated preview: strokes draw themselves
  in command order, travel moves appear as dashed lines, and a marker
  follows the tool. Open it in a web browser; reload to replay.

### Usage

```bash
# Demo v1 (A4, pen) — uses tool_width_mm/overlap/inset from that config
python3 scripts/generate_painting_paths.py --config configs/demo_v1_a4_pen.json

# Old 12in square colored profile
python3 scripts/generate_painting_paths.py --config configs/mondrian_12x12_paint.json
```

`--config` defaults to `configs/demo_v1_a4_pen.json`, same as
`mondrian_generator.py`. Requires `output/<painting_plan_file>` (per
that config) to already exist — run `mondrian_generator.py` with the
*same* `--config` first. Every run prints a confirmation for each file, e.g.:

```
Generated output/painting_paths.json
Generated output/path_preview.svg
Generated output/path_animation.svg (~44s animation — open in a web browser, reload to replay)
Validation passed (0 warnings).
```

### How it works

1. `load_painting_plan()` reads the plan at `output/<painting_plan_file>`
   (from the config), raising a clear error pointing at
   `mondrian_generator.py` if it's missing.
2. `build_commands()` walks the plan's `operations` list in the order
   it's already in (so grid lines that are already last stay last), and
   converts each one using `path_generation` settings from the config:
   - `rectangle_to_commands()` insets the rectangle by `edge_inset_mm`,
     computes horizontal stripe row centers via
     `compute_stripe_row_centers()` (spaced using `tool_width_mm` and
     `stroke_overlap_ratio`), and emits alternating left-to-right /
     right-to-left (boustrophedon) `paint_stroke` commands per row.
     Rectangles too small to paint after the inset are skipped. (In
     monochrome mode there are no `paint_rectangle` operations to begin
     with, so this path is unused.)
   - `line_to_commands()` emits one `select_tool -> dip_paint ->
     move_to -> lower_tool -> paint_stroke -> lift_tool` sequence per
     line.
3. `build_painting_paths()` assembles the JSON: project/style/version,
   a `config` block (profile name + config path), `source_file` (the
   plan path read), canvas metadata (copied from the plan), the
   `path_generation` settings from the config, the full `commands` list,
   and a `debug` summary (command/region counts, estimated total paint
   travel distance in mm).
4. `render_svg()` draws the same commands: paint strokes as
   semi-transparent colored lines (so overlapping stripes show up as
   visibly darker bands) and travel moves as dashed gray lines, so the
   boustrophedon order is actually visible instead of looking like one
   solid block.
5. `render_animated_svg()` renders the same commands as a
   self-contained animated SVG (SMIL, no JavaScript):
   `build_animation_timeline()` assigns each stroke/travel a start time
   and duration from constant preview speeds
   (`ANIMATION_PAINT_SPEED_MM_S`, `ANIMATION_TRAVEL_SPEED_MM_S`, plus a
   short `ANIMATION_TOOL_PAUSE_S` for lower/lift/dip), then each stroke
   draws itself in order over a faint underlay of the finished artwork,
   with a round marker following the tool (solid while lowered, faded
   while lifted). Works for any profile since it only reads the command
   list — pen lines and colored boustrophedon fills animate the same
   way. These speeds are visual pacing only, not robot motion
   parameters.
6. `main()` loads and validates the config, creates `output/` if
   needed, loads the plan, builds the command list, runs it through
   `path_validation.validate_painting_paths()` and stores the result
   under `painting_paths["validation"]`, then writes all three files.

### Validation

Before writing `painting_paths.json`, `main()` calls
`validate_painting_paths()` from [`path_validation.py`](#path_validationpy)
and adds the result as a top-level `"validation"` key:

```json
"validation": { "passed": true, "errors": [], "warnings": [] }
```

The result is also printed to the console, and the script exits with a
non-zero status when validation reports errors (output files are still
written first so a failing run can be inspected). See
`docs/painting-paths-format.md` for the full rule set and
`path_validation.py` below for the implementation.

### Debug summary fields

`painting_paths.json["debug"]` includes:

| Field | Meaning |
| --- | --- |
| `num_commands` | Total commands generated. |
| `num_paint_stroke_commands` | Number of `paint_stroke` commands. |
| `estimated_total_paint_distance_mm` | Sum of Euclidean stroke lengths. |
| `num_fill_regions` | Number of `paint_rectangle` operations in the source plan. |
| `num_grid_lines` | Number of `paint_line` operations in the source plan. |
| `num_select_tool_commands` | Number of `select_tool` commands. |
| `num_lift_tool_commands` | Number of `lift_tool` commands. |
| `num_lower_tool_commands` | Number of `lower_tool` commands. |
| `num_dip_paint_commands` | Number of `dip_paint` commands. |
| `num_move_to_commands` | Number of `move_to` commands that are *pure travel* — i.e. **not** immediately followed by `lower_tool`. Under the current stripe/line generation, every `move_to` positions the tool right before painting, so this is normally `0`. |

### Config fields used

`path_generation.tool_width_mm`, `.stroke_overlap_ratio`, and
`.edge_inset_mm` replace the old hardcoded `TOOL_WIDTH_MM`,
`STROKE_OVERLAP_RATIO`, `EDGE_INSET_MM` constants — see
[Important config fields](#important-config-fields) above.

`STROKE_PREVIEW_OPACITY`, `TRAVEL_LINE_COLOR`, `TRAVEL_LINE_WIDTH_MM`,
and the `ANIMATION_*` pacing constants remain top-of-file constants in
this script: they only affect the preview/animation SVGs' appearance,
not the robot path data, so they aren't part of the config.

### Command primitives

Each entry in `painting_paths.json["commands"]` is one of:

| Command | Key fields | Meaning |
| --- | --- | --- |
| `select_tool` | `color` | Switch to the tool/brush for this color. |
| `dip_paint` | `color` | Reload paint before the next stroke(s). |
| `move_to` | `x_mm`, `y_mm` | Travel move with the tool lifted. |
| `lower_tool` | — | Put the tool down on the canvas. |
| `paint_stroke` | `color`, `from_mm`, `to_mm` | Drag the tool in a straight line while painting. |
| `lift_tool` | — | Raise the tool off the canvas. |

Every command also carries a `label` tying it back to the source
operation (e.g. `blue_block_1_row3`), for debugging. This is still an
intermediate representation, not real robot motor commands — no actual
motor coordinates, timing, or hardware I/O yet, and not direct Aubo i5
motor code.

Full format reference (coordinate system, command fields, assumptions,
validation rules): `docs/painting-paths-format.md`.

### Ideas for future features

- Vertical or diagonal fill strategies, selectable per rectangle
  (matching a future `fill_strategy` field in the plan).
- Smarter travel ordering (e.g. nearest-neighbor) instead of following
  plan order exactly, once travel time matters.
- Translate `painting_paths.json` into actual robot motion/G-code, once
  a robot calibration config (e.g. `configs/aubo_i5_lab_setup.json`)
  exists.

---

## generate_test_line.py

Generates a `painting_paths.json`-format file containing a **single
straight test line** — by default the 50 mm first-contact line from
`docs/hardware-test-checklist.md` section 9 (from `(80, 140)` to
`(130, 140)` on A4). This way the very first pen stroke on hardware
exercises the exact same file format and future robot adapter as the
real generated artwork, instead of a hand-entered line on the robot side.

### Usage

```bash
# Default 50 mm checklist line, Demo v1 config
python3 scripts/generate_test_line.py

# Custom line
python3 scripts/generate_test_line.py --start 80 140 --end 130 140
```

### Options

| Flag | Description |
| --- | --- |
| `--config PATH` | Pipeline config (canvas size, tool settings, output directory). Defaults to `configs/demo_v1_a4_pen.json`. |
| `--start X_MM Y_MM` | Line start point in mm. Default `80 140`. |
| `--end X_MM Y_MM` | Line end point in mm. Default `130 140`. |

### Outputs

Written to the config's `output.directory`, with fixed names so they can
never overwrite the real artwork outputs:

```text
output/test_line_paths.json     Single-stroke painting_paths-format file
output/test_line_preview.svg    Visual preview of the test line
```

The file is validated with `path_validation.validate_painting_paths()`
exactly like real artwork paths; the result is stored under
`"validation"`, printed to the console, and the script exits non-zero if
validation fails (e.g. the line leaves the canvas).

---

## path_validation.py

Importable module (no CLI) that validates `painting_paths.json`-style
data: structurally correct commands, coordinates within canvas bounds,
and safe enough for early software/hardware review. Used by
`generate_painting_paths.py` (see [Validation](#validation) above) and
safe to import from other scripts or tests. Works for any canvas size
(not assumed square) since bounds checks use `width_mm`/`height_mm`
independently.

### Usage

```python
from path_validation import validate_painting_paths

result = validate_painting_paths(painting_paths)
# {"passed": bool, "errors": [...], "warnings": [...]}
```

`passed` is `true` iff `errors` is empty; `warnings` never affect it.

### Functions

| Function | Purpose |
| --- | --- |
| `validate_painting_paths(painting_paths)` | Main entry point: validates canvas + every command, returns the result dict. |
| `validate_canvas(canvas)` | Checks canvas exists, `width_mm`/`height_mm` are present and positive (error), `origin` is `"top-left"` (warning otherwise). Returns `(errors, warnings)`. |
| `validate_command(command, index, canvas)` | Checks one command's structure against its type's rules (see `docs/painting-paths-format.md`), including that `move_to` and `paint_stroke` coordinates stay inside canvas bounds — the robot physically travels to `move_to` targets, so those are bounds-checked too. Unknown command types warn, not error. Returns `(errors, warnings)`. |
| `point_inside_canvas(point, canvas)` | `True` if `0 <= x <= width_mm` and `0 <= y <= height_mm`. |
| `is_number(value)` | `True` for `int`/`float`, explicitly `False` for `bool`. |
| `stroke_distance(from_point, to_point)` | Euclidean distance between two `[x, y]` points. |

Full rule set (what's an error vs. a warning for each command type):
`docs/painting-paths-format.md`.

### Ideas for future features

- A CLI wrapper to validate an arbitrary `painting_paths.json` file
  on disk without importing it from another script.
- Check that every `paint_stroke` is bracketed by `lower_tool`/`lift_tool`.
- Warn on strokes shorter than the tool width (likely a dot, not a stroke).
