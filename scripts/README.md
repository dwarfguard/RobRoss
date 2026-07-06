# Scripts

Utility scripts for this repo. Each script gets its own section below —
when adding a new script, copy the section format used for
`mondrian_generator.py`.

---

## mondrian_generator.py

Generates a randomized Mondrian/De Stijl-style SVG: a 12in x 12in canvas
recursively subdivided into cells, a few of which are filled with the
classic red/yellow/blue (and occasionally gray) accents, separated by
thick black grid lines.

### Usage

```bash
# Random graphic, new layout every run
python3 scripts/mondrian_generator.py

# Reproduce a specific graphic
python3 scripts/mondrian_generator.py --seed 2124073818
```

Output is written to `output/mondrian_preview.svg` (directory is created
if missing).

Every run prints the seed used, e.g.:

```
Generated output/mondrian_preview.svg (seed=2124073818)
```

Copy that seed value into `--seed` to regenerate the exact same graphic
later.

### Options

| Flag | Description |
| --- | --- |
| `--seed N` | Use a fixed integer seed for reproducible output. Omit for a random seed (printed after generation). |

### How it works

1. `subdivide()` recursively splits the canvas rectangle into two
   (vertical or horizontal cut, random position) until cells hit a
   minimum size or a depth-scaled stop probability triggers. This
   produces the leaf cells and the internal grid lines in one pass.
2. `generate_mondrian_svg()` picks 2-4 leaf cells at random to fill with
   accent colors (rest stay white via the background rect), picks a
   random stroke width for the grid/border lines, and assembles the SVG
   string.
3. `main()` resolves/reports the seed and writes the file.

### Tuning knobs (top of file)

| Constant | Effect |
| --- | --- |
| `CANVAS_SIZE_MM` | Physical canvas size (defaults to 12in square). |
| `ACCENT_COLORS` | Palette used for colored blocks. |
| `NEUTRAL_ACCENT` | Extra gray accent, added to the palette ~25% of the time. |
| `MIN_CELL_FRACTION` | Smallest allowed cell edge, as a fraction of canvas size. Lower = smaller/more numerous cells possible. |
| `MAX_SPLIT_DEPTH` | Hard cap on recursion depth. |

Accent count (2-4), stroke width (5-8mm), and the per-depth stop
probability are inline in `generate_mondrian_svg()` / `subdivide()`
rather than top-level constants — pull them up if they need to become
tunable too.

### Ideas for future features

- CLI flags for canvas size, min cell fraction, accent count range, and
  stroke width range (avoid hardcoding once more than one param needs
  tuning).
- Export to PNG/PDF alongside SVG.
- Alternate palettes (e.g. monochrome, pastel) selectable via flag.
- Batch mode: generate N variations at once into `output/`.
