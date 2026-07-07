# Scripts

Utility scripts for this repo. Each script gets its own section below —
when adding a new script, copy the section format used for
`mondrian_generator.py`.

---

## mondrian_generator.py

Generates a randomized Mondrian/De Stijl-style layout: a 12in x 12in
(304.8mm x 304.8mm) canvas recursively subdivided into cells, a few of
which are filled with the classic red/yellow/blue (and occasionally
gray) accents, separated by thick black grid lines.

One run produces two outputs from the *same* in-memory layout:

- `output/mondrian_preview.svg` — human preview image.
- `output/painting_plan.json` — robot-friendly painting plan (ordered
  paint operations in millimeters, top-left origin, x right / y down).

Because both files are built from the same generated rectangles and
lines, the SVG and the JSON always describe the identical artwork.

### Usage

```bash
# Random graphic, new layout every run
python3 scripts/mondrian_generator.py

# Reproduce a specific graphic
python3 scripts/mondrian_generator.py --seed 2124073818
```

Output is written to `output/mondrian_preview.svg` and
`output/painting_plan.json` (directory is created if missing).

Every run prints the seed used and confirms each file, e.g.:

```
Generated output/mondrian_preview.svg (seed=2124073818)
Generated output/painting_plan.json (seed=2124073818)
```

Copy that seed value into `--seed` to regenerate the exact same graphic
and plan later.

### Options

| Flag | Description |
| --- | --- |
| `--seed N` | Use a fixed integer seed for reproducible output. Omit for a random seed (printed after generation). |

### How it works

1. `subdivide()` recursively splits the canvas rectangle into two
   (vertical or horizontal cut, random position) until cells hit a
   minimum size or a depth-scaled stop probability triggers. This
   produces the leaf cells and the internal grid lines in one pass.
2. `generate_mondrian_layout()` picks 2-4 leaf cells at random to fill
   with accent colors (rest stay white via the background rect), picks a
   random stroke width for the grid/border lines, labels every
   rectangle/line (e.g. `red_block_1`, `grid_line_3`, `border_top`), and
   returns the `Rect`/`Line` lists — the single source of truth for both
   outputs.
3. `render_svg()` turns those rectangles and lines into the SVG string.
4. `build_painting_plan()` turns the same rectangles and lines into the
   JSON painting plan: only non-white rectangles become
   `paint_rectangle` operations, followed by all lines as `paint_line`
   operations (grid lines painted last so they clean up rectangle
   edges), plus canvas/coordinate metadata and a `debug` summary.
5. `main()` resolves/reports the seed, builds the layout once, and
   writes both files from it.

### Tuning knobs (top of file)

| Constant | Effect |
| --- | --- |
| `CANVAS_SIZE_MM` | Physical canvas size (defaults to 12in square). |
| `ACCENT_COLORS` | Palette used for colored blocks. |
| `NEUTRAL_ACCENT` | Extra gray accent, added to the palette ~25% of the time. |
| `MIN_CELL_FRACTION` | Smallest allowed cell edge, as a fraction of canvas size. Lower = smaller/more numerous cells possible. |
| `MAX_SPLIT_DEPTH` | Hard cap on recursion depth. |

Accent count (2-4), stroke width (5-8mm), and the per-depth stop
probability are inline in `generate_mondrian_layout()` / `subdivide()`
rather than top-level constants — pull them up if they need to become
tunable too.

### Painting plan JSON

`painting_plan.json` (see `build_painting_plan()`) includes:

- `canvas` — size in mm and inches, origin corner.
- `coordinate_system` — x/y axis directions.
- `assumptions` — plain-language notes about paint order and canvas
  starting state, for anyone (human or downstream code) consuming the
  plan.
- `operations` — ordered list of `paint_rectangle` (colored blocks,
  solid fill) then `paint_line` (grid lines, then outer border) steps,
  each with a `label` for debugging.
- `debug` — seed used, rectangle/line/operation counts, and the list of
  colors used.

This is an intermediate plan, not robot motor code — it doesn't cover
things like brush lift/lower, travel paths between operations, or
paint mixing.

### Ideas for future features

- CLI flags for canvas size, min cell fraction, accent count range, and
  stroke width range (avoid hardcoding once more than one param needs
  tuning).
- Export to PNG/PDF alongside SVG.
- Alternate palettes (e.g. monochrome, pastel) selectable via flag.
- Batch mode: generate N variations at once into `output/`.
- Real fill strategies (e.g. `horizontal_stripes`) instead of the
  current `solid_fill` placeholder, once the robot supports them.

---

## generate_painting_paths.py

Converts the abstract paint operations in `painting_plan.json` into
concrete robot-style stroke path commands (horizontal stripe fills for
rectangles, single strokes for lines). This script is read-only with
respect to the Mondrian layout: it never generates a new random layout,
it only reads the plan that `mondrian_generator.py` already produced.

One run produces two outputs from the *same* command list:

- `output/painting_paths.json` — ordered stroke commands for a robot to
  follow.
- `output/path_preview.svg` — visual preview of those stroke paths.

### Usage

```bash
python3 scripts/generate_painting_paths.py
```

Requires `output/painting_plan.json` to already exist (run
`mondrian_generator.py` first). Every run prints a confirmation for each
file, e.g.:

```
Generated output/painting_paths.json
Generated output/path_preview.svg
```

### How it works

1. `load_painting_plan()` reads `output/painting_plan.json`, raising a
   clear error pointing at `mondrian_generator.py` if it's missing.
2. `build_commands()` walks the plan's `operations` list in the order
   it's already in (so grid lines that are already last stay last), and
   converts each one:
   - `rectangle_to_commands()` insets the rectangle by `EDGE_INSET_MM`,
     computes horizontal stripe row centers via
     `compute_stripe_row_centers()` (spaced using `TOOL_WIDTH_MM` and
     `STROKE_OVERLAP_RATIO`), and emits alternating left-to-right /
     right-to-left (boustrophedon) `paint_stroke` commands per row.
     Rectangles too small to paint after the inset are skipped.
   - `line_to_commands()` emits one `select_tool -> dip_paint ->
     move_to -> lower_tool -> paint_stroke -> lift_tool` sequence per
     line.
3. `build_painting_paths()` assembles the JSON: project/style/version,
   `source_file`, canvas metadata (copied from the plan),
   `path_settings`, the full `commands` list, and a `debug` summary
   (command/region counts, estimated total paint travel distance in mm).
4. `render_svg()` draws the same commands: paint strokes as
   semi-transparent colored lines (so overlapping stripes show up as
   visibly darker bands) and travel moves as dashed gray lines, so the
   boustrophedon order is actually visible instead of looking like one
   solid block.
5. `main()` creates `output/` if needed, loads the plan, builds the
   command list, runs it through `path_validation.validate_painting_paths()`
   and stores the result under `painting_paths["validation"]`, then writes
   both files.

### Validation

Before writing `painting_paths.json`, `main()` calls
`validate_painting_paths()` from [`path_validation.py`](#path_validationpy)
and adds the result as a top-level `"validation"` key:

```json
"validation": { "passed": true, "errors": [], "warnings": [] }
```

This does not stop the script from writing output on failure — it's a
review aid, not a gate. See `docs/painting-paths-format.md` for the full
rule set and `path_validation.py` below for the implementation.

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

### Tuning knobs (top of file)

| Constant | Effect |
| --- | --- |
| `TOOL_WIDTH_MM` | Width of a single paint stroke/stripe. |
| `STROKE_OVERLAP_RATIO` | How much each stripe overlaps the previous one (0-1). |
| `EDGE_INSET_MM` | How far inside a rectangle's edge strokes are kept. |
| `STROKE_PREVIEW_OPACITY` | Preview-only: stroke transparency in `path_preview.svg`, so overlaps are visible. |
| `TRAVEL_LINE_COLOR` / `TRAVEL_LINE_WIDTH_MM` | Preview-only: styling of the dashed tool-travel lines. |

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
motor coordinates, timing, or hardware I/O yet.

Full format reference (coordinate system, command fields, assumptions,
validation rules): `docs/painting-paths-format.md`.

### Ideas for future features

- CLI flags to point at a different input plan / output location.
- Vertical or diagonal fill strategies, selectable per rectangle
  (matching a future `fill_strategy` field in the plan).
- Smarter travel ordering (e.g. nearest-neighbor) instead of following
  plan order exactly, once travel time matters.
- Translate `painting_paths.json` into actual robot motion/G-code.
- Have `main()` fail loudly (non-zero exit) when validation reports
  errors, instead of only recording them under `"validation"`.

---

## path_validation.py

Importable module (no CLI) that validates `painting_paths.json`-style
data: structurally correct commands, coordinates within canvas bounds,
and safe enough for early software/hardware review. Used by
`generate_painting_paths.py` (see [Validation](#validation) above) and
safe to import from other scripts or tests.

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
| `validate_command(command, index, canvas)` | Checks one command's structure against its type's rules (see `docs/painting-paths-format.md`). Unknown command types warn, not error. Returns `(errors, warnings)`. |
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
