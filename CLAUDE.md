# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

RobRoss is a robot-arm art project (Aubo i5). The active target is **Demo v1**: prove the robot
can reliably draw a preprocessed Mondrian-style line drawing with a pen on A4 paper — no color,
no paint, no real-time AI. Read `README.md` first; it's the authoritative project overview
(bilingual EN/中文) and is kept up to date with current scope/non-goals. `AGENTS.md` points to the
same set of "read these first" docs.

Treat Demo v1 A4 pen drawing as the active requirement. Do not treat the legacy 12-inch colored
Mondrian behavior as current scope unless explicitly asked — it's preserved for backward
compatibility only (`configs/mondrian_12x12_paint.json`).

## Software pipeline

```
Config profile (configs/*.json)
  -> Image_Process/mondrian/mondrian_generator.py    -> output/painting_plan.json (+ preview SVG)
  -> Image_Process/mondrian/generate_painting_paths.py -> output/painting_paths.json (+ preview/animated SVG)
  -> ros2/robross_painter (painting_executor.cpp)     -> MoveIt motion on Aubo i5
```

Both Python scripts are config-driven (`--config`, defaults to `configs/demo_v1_a4_pen.json`) —
**always pass the same `--config` to both scripts in a run**; `generate_painting_paths.py` reads
the `painting_plan.json` that `mondrian_generator.py` wrote for that same config, and mismatched
configs silently mix profiles or fail to find the plan.

`painting_plan.json` and `painting_paths.json` are both intermediate representations in
millimeters (origin top-left, x right, y down) — **not** direct robot motor code. The ROS 2
adapter (`ros2/robross_painter`) is the only place that translates these into actual robot motion,
and robot calibration (taught paper corners, safe/contact Z height, tool offsets, home pose) lives
there (`config/demo_v1_rviz.yaml`), never inside the artwork/path configs. Preserve this
separation between artwork generation, path generation, validation, and robot execution.

There is a second, independent artwork-generation route in `Image_Process/sketch/`: instead of a
procedurally generated vector design, it traces edges out of an arbitrary source bitmap (Canny
edge detection + skeletonization) and goes straight to `painting_paths.json` — no intermediate
`painting_plan.json`, since there's no vector design to describe first. It emits the exact same
command vocabulary as the mondrian route, so it's consumable by the same
`ros2/robross_painter` executor. See `Image_Process/sketch/README.md`.

There is a third route in `Image_Process/image_to_mondrian/`: also takes an arbitrary source
photo, but quantizes it to a fixed 5-color palette (the robot's actual pens — red/blue/yellow/
black/white by default), segments same-color pixels into connected regions, and **fully fills**
each region (not just outlines) plus classic Mondrian black grid lines between color blocks — a
non-ML color-quantization + connected-component pipeline, chosen specifically because it can
guarantee gap-free fill without the risk of vectorizing an ML model's raster output. Same
`painting_paths.json` command vocabulary, same executor. See
`Image_Process/image_to_mondrian/README.md`.

## Commands

Run everything from the repo root (scripts use CWD-relative `configs/`/`output/` paths).

```bash
# Generate Demo v1 (A4, pen, monochrome) artwork plan — new random layout each run
python3 Image_Process/mondrian/mondrian_generator.py --config configs/demo_v1_a4_pen.json --seed 123

# Convert that plan into robot-style stroke path commands
python3 Image_Process/mondrian/generate_painting_paths.py --config configs/demo_v1_a4_pen.json

# Generate the single 50mm first-contact test line (docs/hardware-test-checklist.md section 9)
python3 Image_Process/mondrian/generate_test_line.py

# Run the full test suite
python3 -m unittest discover Image_Process/mondrian/tests

# Run a single test file / test case
python3 -m unittest Image_Process.mondrian.tests.test_path_validation
python3 -m unittest Image_Process.mondrian.tests.test_path_validation.SomeTestClass.test_method
```

Same `--seed` + config produces deterministic output. Omit `--seed` for a random one (printed
after generation, so it can be copied back in to reproduce that exact layout).

The mondrian route is pure standard library — no `requirements.txt`/`pyproject.toml`, no install
step. The sketch route needs third-party image libraries (see below).

For the legacy 12-inch colored profile, substitute `configs/mondrian_12x12_paint.json` for
`configs/demo_v1_a4_pen.json` in the same two commands.

```bash
# Sketch route: trace configs/sketch_demo_a4.json's source image straight to painting_paths.json
# Needs opencv-python + numpy + scikit-image — see Image_Process/sketch/README.md for the apt install
python3 Image_Process/sketch/generate_sketch_paths.py --config configs/sketch_demo_a4.json

# image_to_mondrian route: quantize + fill configs/image_to_mondrian_demo_a4.json's source photo
# Needs opencv-python + numpy (no scikit-image) — see Image_Process/image_to_mondrian/README.md
python3 Image_Process/image_to_mondrian/generate_painting_paths.py --config configs/image_to_mondrian_demo_a4.json
```

### ROS 2 side (`ros2/robross_painter`)

Built as a normal ROS 2 (Humble) package inside a colcon workspace alongside the
RobRoss-maintained Aubo driver fork (`ros2/robross_aubo.repos` is the vcstool manifest). Full
workspace bootstrap and three-terminal fake-hardware RViz flow is documented in `README.md`
("ROS 2 Aubo Setup") and `ros2/robross_painter/README.md` — don't duplicate it here. Key points
if touching this package:

- `painting_executor` (`src/painting_executor.cpp`) reads a `painting_paths.json`-format file and
  maps each command to MoveIt motion (see command table in `ros2/robross_painter/README.md`).
- Do not spin the executor node in an external executor — `MoveGroupInterface` spins the node
  internally in MoveIt Humble, and a second executor can steal its action responses. For the same
  reason the code builds the retiming start state from the trajectory's first waypoint instead of
  calling `getCurrentState()`.
- Safety checks refuse out-of-canvas coordinates, `move_to` with the pen down, and strokes with
  the pen up.

## Architecture within `Image_Process/mondrian/`

- **`config_loader.py`** — `load_config(path)` / `validate_config(config)`. Shared validation so
  the two main scripts can't drift on what counts as a valid config (required sections `canvas`,
  `artwork`, `path_generation`, `output`; positive canvas size; `origin == "top-left"`;
  `0 <= stroke_overlap_ratio < 1`; etc). Raises `ConfigError` (a `ValueError`) listing *every*
  problem found, not just the first.

- **`mondrian_generator.py`** — `subdivide()` recursively splits the canvas into leaf cells
  (vertical/horizontal cuts, depth-scaled stop probability) in one pass, producing both leaf
  rectangles and the internal grid lines. `generate_mondrian_layout()` then styles those per
  `artwork.palette_mode`: `"monochrome"` (Demo v1) skips accent-cell selection entirely — no
  `paint_rectangle` ops, grid lines + border only; `"color"` picks 2-4 accent-colored cells.
  `render_svg()` and `build_painting_plan()` both consume the *same* generated rectangle/line
  list, so the SVG preview and the JSON plan always describe identical artwork.

- **`generate_painting_paths.py`** — read-only with respect to the layout (never generates new
  random geometry, only converts an existing plan). `build_commands()` walks the plan's
  `operations` in existing order; `rectangle_to_commands()` insets by `edge_inset_mm` and emits
  boustrophedon (alternating direction) `paint_stroke` rows spaced by `tool_width_mm` /
  `stroke_overlap_ratio`; `line_to_commands()` emits one
  `select_tool -> dip_paint -> move_to -> lower_tool -> paint_stroke -> lift_tool` sequence per
  line. Also renders a static preview SVG and a self-contained SMIL animated SVG
  (`render_animated_svg()` — visual pacing constants only, not robot motion parameters). Before
  writing output, runs `path_validation.validate_painting_paths()` and stores the result under
  `painting_paths["validation"]`; exits non-zero on validation errors (files are still written so
  a failing run can be inspected).

- **`path_validation.py`** — importable, no CLI. `validate_painting_paths(painting_paths)` returns
  `{"passed": bool, "errors": [...], "warnings": [...]}`; `passed` is true iff `errors` is empty.
  Bounds-checks both `move_to` and `paint_stroke` coordinates against canvas size (the robot
  physically travels to `move_to` targets too, so those need the same out-of-bounds error
  treatment as strokes). Unknown command types warn rather than error, so new command types don't
  break existing pipelines. Full rule set: `docs/painting-paths-format.md`.

- **`generate_test_line.py`** — generates a `painting_paths.json`-format file with a single
  straight line (default: the 50mm first-contact line from
  `docs/hardware-test-checklist.md`), so the very first hardware pen stroke exercises the same
  file format/robot adapter as real generated artwork. Fixed output filenames
  (`test_line_paths.json`/`test_line_preview.svg`) so it can never overwrite real artwork outputs.

- **`tests/`** — `context.py` adds `Image_Process/mondrian/` to `sys.path` (scripts there import
  each other as top-level modules, not a package), matching how the scripts are run from the repo
  root. Import it first in new test files the same way the existing `test_*.py` files do.

## Architecture within `Image_Process/sketch/`

Self-contained, own `config_loader.py`/`path_validation.py` (the latter copied byte-for-byte from
the mondrian one — the `painting_paths.json` command-list format it validates doesn't care how the
commands were generated). `canny_edges.py` does Canny edge detection + skeletonization + a pixel
graph walk to produce one traced centerline per independent line; `path_ordering.py` is a shared
greedy nearest-neighbor stroke sequencer; `generate_sketch_paths.py` maps traced pixel strokes onto
the canvas (aspect-ratio-preserving, centered within `margin_mm`), orders them, and emits one
`move_to -> lower_tool -> paint_stroke (xN) -> lift_tool` sequence per traced line — a curved line
becomes multiple straight `paint_stroke` segments, one per consecutive point pair. Full detail:
`Image_Process/sketch/README.md`.

## Architecture within `Image_Process/image_to_mondrian/`

Self-contained, own `config_loader.py` and a byte-for-byte copy of `path_validation.py` and
`sketch/path_ordering.py`. `color_quantize.py::preprocess()` resizes then smooths the photo —
either Gaussian blur (`blur_kernel_size`) or, preferably for real photos, edge-preserving
`cv2.bilateralFilter` (`bilateral_d`, takes priority when set) which smooths flat/gradient areas
but keeps real high-contrast edges (e.g. an eye against skin) sharp, unlike Gaussian blur.
`quantize_to_palette()` does fully-vectorized nearest-palette-color matching (Lab space by
default) — no per-pixel Python loop. Optional `palette.neutral_chroma_threshold` (Lab only,
default 0/disabled) gates this: pixels below the threshold only match the palette's own neutral
colors (white/black), pixels at or above only match its chromatic ones (red/blue/yellow) — keeps
desaturated mid-tones (skin, hair, shadow) from being forced into a bold color just because
they're marginally closer, so photos land closer to real Mondrian's mostly-white-canvas look
instead of every pixel getting colored in. Prefer `palette.neutral_chroma_percentile` over a
hand-picked `neutral_chroma_threshold` number — `color_quantize.compute_adaptive_chroma_threshold()`
computes the threshold from *that photo's own* Lab chroma distribution, so the same percentile
value generalizes across differently-colored photos instead of being one magic number tuned to a
single image (verified against three visually different photos in
`configs/image_to_mondrian_lenna_a4.json`'s development). `segmentation.py::segment_image()` runs
a per-color morphological close (`morph_close_kernel_px`, optional) *then* open
(`clean_label_image()`, strips single-pixel speckle — close runs first so open doesn't erase a
gap's ragged edges before close can bridge them) then `cv2.connectedComponentsWithStats()` per
color, then drops regions under `min_region_area_mm2`; each kept region dict carries its own
boolean `mask`, consumed directly by both `region_fill.py` (fill) and `border_tracing.py`
(outline). A closing kernel large enough to bridge a real gap in one object (e.g. a hat split by
shadow) is also large enough to merge small nearby semantic features (e.g. an eye) into an
adjacent region, since closing only sees pixel geometry — confirmed this isn't a tuning problem:
an eye's pixel blob is often already connected to nearby hair through eyebrow/eyelash strands
*before* any morphology runs, so no kernel size threads that needle. `face_protection.py`
(optional, needs `mediapipe` — the one narrowly-scoped ML dependency in this otherwise-classical
module) runs a pretrained MediaPipe Face Mesh model and returns a `protected_mask` that
`clean_label_image()` uses to force-restore protected pixels to their pre-morphology
classification after close/open run, regardless of what either decided — lets `morph_close_kernel_px`
be large enough to fix a real object's shape without erasing nearby facial detail. Returns `None`
(not an error) when `mediapipe` isn't installed or no face is detected. `region_fill.py` is the
new polygon-fill algorithm
(mondrian's rectangle boustrophedon only handles rectangles): `erode_mask()` shrinks a region
inward by `mask_erosion_mm` so strokes don't bleed across color boundaries, `find_row_intervals()`
is a vectorized run-length detector handling concave shapes (more than one paintable interval per
row), `compute_stripe_rows()` mirrors mondrian's row-spacing formula in pixel space. No manual
boustrophedon alternation here — travel optimization is deferred entirely to
`path_ordering.order_strokes()`, since a concave shape's per-row interval count isn't fixed.
`border_tracing.py` traces the classic Mondrian black grid lines via `trace_region_contours()` —
`cv2.findContours()` on one region's own mask at a time (one closed loop per outer boundary, one
per hole), **not** a single walk across a global boundary-pixel network like `sketch/canny_edges.py`'s
skeleton graph walk: a busy real photo's color-boundary network has huge numbers of junctions, and
any graph walk must break a new stroke at every one, fragmenting into thousands of tiny strokes
(each one a full robot lift/travel/lower cycle); a single region's own boundary is always just one
or a few closed loops no matter how jagged the pixel boundary is, so per-region tracing sidesteps
that fragmentation entirely (confirmed against a real photo — see the module's README "Known v1
limitations" for the accepted trade-off: shared edges between adjacent regions get traced twice,
not deduplicated). `generate_painting_paths.py::order_and_build_commands()` groups fill strokes by
color first (physical pen changes cost more than travel distance), greedy-orders each group, then
draws the black grid lines **last** — same convention mondrian uses, with the side effect that the
border line (traced along the true, un-eroded boundary) paints over the thin gap `mask_erosion_mm`
leaves between adjacent fills. Full detail: `Image_Process/image_to_mondrian/README.md`.

## Config profiles

Four profiles exist in `configs/`; use `demo_v1_a4_pen.json` unless intentionally exercising the
legacy behavior or one of the photo-input routes:

| Config | Canvas | Route | Notes |
| --- | --- | --- | --- |
| `demo_v1_a4_pen.json` | A4 210x297mm, 10mm margin | mondrian | monochrome (lines only), 1mm pen, 0% overlap |
| `mondrian_12x12_paint.json` | 12in square, no margin | mondrian | red/yellow/blue accents, 10mm brush, 25% overlap |
| `sketch_demo_a4.json` | A4 210x297mm, 20mm margin | sketch | traces `Image_Process/sketch/assets/apple.png`, 1mm pen |
| `image_to_mondrian_demo_a4.json` | A4 210x297mm, 10mm margin | image_to_mondrian | quantizes+fills `Image_Process/image_to_mondrian/assets/sample.jpg` to 5 colors, 3mm pen |

See `Image_Process/mondrian/README.md` / `Image_Process/sketch/README.md` /
`Image_Process/image_to_mondrian/README.md` ("Important config fields" / "Config fields") for the
full field reference and `docs/painting-paths-format.md` for the `painting_paths.json`
command/validation schema (shared by all three routes).

## Repo layout

- `configs/` — pipeline config profiles (see above).
- `docs/` — `Rob_Ross_Prototype_v1.md` (current prototype source of truth),
  `painting-paths-format.md` (path JSON schema), `hardware-test-checklist.md`,
  `Rob_Ross_Discuss.md` (early brainstorming, not current requirements).
- `Image_Process/` — one subfolder per artwork-generation algorithm: `mondrian/` (procedural
  vector design), `sketch/` (Canny-edge tracing of a source image, lines only), and
  `image_to_mondrian/` (photo quantized to a 5-color palette, fully filled, plus black grid
  lines). New generation approaches get their own sibling subfolder rather than growing inside an
  existing one.
- `output/` — generated artifacts (plans, paths, SVG previews). Intentionally committed
  (seed-123 reference samples), not gitignored.
- `ros2/` — `robross_painter` ROS 2 package (MoveIt executor) and the vcstool manifest for the
  Aubo driver fork workspace.
- `CAD/` — SolidWorks 2025 SP5.0 prototype files for hardware (canvas/paint holders).

## Conventions

- Prefer simple, readable Python using the standard library unless a dependency is clearly
  justified — `mondrian/` has zero third-party dependencies. `Image_Process/sketch/` (`opencv-python`,
  `numpy`, `scikit-image`) and `Image_Process/image_to_mondrian/` (`opencv-python`, `numpy` only)
  are deliberate, folder-scoped exceptions for image loading/quantization/edge-detection — don't
  let that spread to other folders without similarly clear justification.
- `Image_Process/image_to_mondrian/face_protection.py`'s `mediapipe` dependency is this repo's
  first ML model dependency, kept intentionally narrow: optional (`segmentation.protect_face_features`,
  default off), pretrained/no-training/CPU-only, and imported lazily inside the one function that
  needs it so the rest of the module has no hard dependency on it. It exists because a purely
  classical fix was verified not to exist for its specific problem (see the module's README "Why
  (mostly) non-ML") — don't reach for an ML dependency elsewhere in this repo without similarly
  confirming the classical approach has a real ceiling, not just "would be easier."
- Update the relevant Markdown (`README.md`, `Image_Process/mondrian/README.md`,
  `Image_Process/sketch/README.md`, `Image_Process/image_to_mondrian/README.md`,
  `docs/painting-paths-format.md`) whenever behavior or project decisions change; these docs are
  treated as living references, not one-off notes.
