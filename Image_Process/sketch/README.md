# sketch

Image-to-robot-path pipeline for the **sketch / outline-tracing route**:
turns an arbitrary source image into a `painting_paths.json`-format file by
guessing edges from the bitmap, instead of the `mondrian/` route's
procedurally generated vector design. Ported from the `raymond` branch's
`scripts/canny.py` + `scripts/sketch_robot_path.py`, restructured to match
this repo's config-driven, `Image_Process/<algorithm>/` layout and to emit
the same `painting_paths.json` command vocabulary the ROS 2 executor
(`ros2/robross_painter`) already understands, instead of that branch's own
bespoke `{"canvas_size", "tools": [...]}` shape.

**Only this route needs third-party dependencies** — `opencv-python`,
`numpy`, and `scikit-image` for Canny edge detection and skeletonization.
`mondrian/` stays pure standard library; this is a deliberate,
folder-scoped exception (see root `CLAUDE.md`). On this project's Ubuntu
22.04 / ROS 2 Humble setup, all three are plain apt packages:

```bash
sudo apt-get install -y python3-opencv python3-numpy python3-skimage
```

(`python3-opencv` and `python3-numpy` are commonly already present as ROS
dependencies; `python3-skimage` usually needs installing explicitly.)

## Pipeline

```
Config profile (configs/*.json)
  -> canny_edges.extract_strokes()   Canny edges -> skeleton -> traced centerline strokes
  -> path_ordering.order_strokes()   greedy nearest-neighbor travel order
  -> generate_sketch_paths.py        -> output/<config-name>/<painting_paths_file> (+ preview SVG)
```

Unlike `mondrian/`, there is no intermediate `painting_plan.json` step —
there's no vector design to describe first, so `generate_sketch_paths.py`
goes straight from traced pixel strokes to the final command list.

### Usage

```bash
python3 Image_Process/sketch/generate_sketch_paths.py --config configs/sketch_demo_a4.json
```

`--config` defaults to `configs/sketch_demo_a4.json`. Run from the repo
root (paths in the config are CWD-relative), same as the mondrian scripts.

### Config fields

| Field | Meaning |
| --- | --- |
| `canvas.width_mm` / `height_mm` / `margin_mm` / `origin` | Same meaning as the mondrian config schema — `origin` must be `"top-left"`, `margin_mm` keeps the traced drawing inside the paper edge. |
| `source_image.path` | Path to the source bitmap (grayscale is read regardless of the file's color mode). |
| `source_image.canny_threshold1` / `canny_threshold2` | `cv2.Canny()` sensitivity — lower = more detail/noise, higher = only strong edges. |
| `source_image.simplify_epsilon_ratio` | Douglas-Peucker simplification strength as a fraction of each stroke's arc length — larger = fewer points/straighter lines. |
| `path_generation.tool_width_mm` | Pen stroke width (preview rendering + `path_settings.tool_width_mm` in the output). |
| `path_generation.home_position_mm` | Where the pen starts before the first stroke; affects greedy-ordering's first pick. Optional, defaults to the canvas's top-left margin corner. |
| `output.directory`, `.painting_paths_file`, `.preview_svg_file` | Same meaning as the mondrian config schema. |

## canny_edges.py

`extract_strokes(image_path, threshold1, threshold2)`: reads the image
grayscale, runs Canny edge detection, then `skimage.morphology.skeletonize`
to thin edges to single-pixel centerlines (avoiding the mirrored-duplicate
problem `cv2.findContours` has when it walks both sides of a stroke as one
blob boundary). Walks the skeleton pixel graph directly (endpoints/
junctions as nodes) to produce one entry per independent line — pixel
`(x, y)` points plus whether it's a closed loop (e.g. small blemish rings
in a source image). Returns `(strokes, image_size)`.

`simplify(points_xy, closed, epsilon_ratio)`: Douglas-Peucker point
reduction (`cv2.approxPolyDP`) on one stroke's pixel points.

**Note when swapping source images:** works best with a clean-background,
high-contrast image — busy backgrounds or low contrast flood the traced
strokes with tiny noise loops from Canny.

## path_ordering.py

Shared greedy nearest-neighbor stroke ordering (ported unchanged from the
`raymond` branch's `scripts/path_ordering.py`). `order_strokes(strokes_data,
home_position)` walks strokes starting from `home_position`, each time
picking whichever remaining stroke has an endpoint closest to the pen's
current position (reversing open strokes if that end is closer), to cut
down total pen-up travel. `total_travel_distance(strokes_points,
home_position)` sums the pen-up jumps for a given ordering — used to report
the before/after travel reduction.

## generate_sketch_paths.py

`build_canvas_strokes()` runs `extract_strokes()` + `simplify()` on the
config's source image, then scales pixel points onto the canvas's drawable
box (canvas size minus `margin_mm` on each side), preserving aspect ratio
and centering the result — canvas and image both use a top-left, y-down
origin, so no axis flip is needed (unlike the `raymond` branch's
`"bottom-left"` option, which this route doesn't need since
`config_loader.py` requires `"top-left"`, same as `mondrian/`).

`main()` orders the strokes, then `build_commands()` turns each ordered
stroke into one `move_to -> lower_tool -> paint_stroke (x N) -> lift_tool`
sequence — one straight `paint_stroke` per consecutive point pair, since a
curved traced line becomes multiple straight strokes (see
`docs/painting-paths-format.md`). A single `select_tool`/`dip_paint`
(black) is emitted once up front, since this route only ever produces one
monochrome pen tool.

Before writing output, runs `path_validation.validate_painting_paths()` —
the exact same validator `mondrian/generate_painting_paths.py` uses (copied
unchanged into this folder for self-containment) — and stores the result
under `painting_paths["validation"]`; exits non-zero on validation errors,
same convention as the mondrian route.

## path_validation.py

Copied unchanged from `Image_Process/mondrian/path_validation.py` — the
`painting_paths.json` command-list validator is generic to the command
format, not to how the commands were generated, so both routes share it
byte-for-byte rather than importing across folders (keeping this folder
self-contained per `Image_Process/README.md`).

## assets/apple.png

Reference source image, carried over from the `raymond` branch's
`assets/apple.png` — clean background, high contrast, good for exercising
the pipeline end to end.

## Ideas for future features

- CLI flags / config fields for a raw Canny-edge-bitmap debug output
  (the `raymond` branch's `canny.py` wrote one), useful for tuning
  thresholds without re-running the full stroke walk.
- Batch mode: run multiple source images through one config.
- A smoothed-curve preview mode (Catmull-Rom, like the `raymond` branch's
  SVG output) purely for human preview — the actual `painting_paths.json`
  stays straight-segment, since that's what the robot executor understands.
