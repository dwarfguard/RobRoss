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
- Translate `painting_plan.json` into actual robot motion/G-code.
