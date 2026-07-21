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

Four independent artwork-generation routes, all producing the same `painting_paths.json` command
vocabulary consumed by `ros2/robross_painter`:

```
Route                      Input          Steps                                      Output
─────────────────────────────────────────────────────────────────────────────────────────────────
mondrian/                  (none)         subdivide → style → plan → paths            painting_plan.json → painting_paths.json
sketch/                    source photo   Canny → skeletonize → trace → paths         painting_paths.json
image_to_mondrian/         source photo   quantize → segment → fill → border → paths  painting_paths.json
gemini_mondrian/           source photo   Gemini API → quantize → segment → fill →    painting_paths.json
                                           border → paths
```

Every config's `output.directory` points at its own `output/<config-name>/` subfolder (named after
the config file, minus `.json`) — this is what keeps different profiles' generated files from
overwriting each other (demo_v1_a4_pen and mondrian_12x12_paint used to share the same literal
output filenames and clobbered one another before each config got its own subfolder). After
generating one or more configs, `python3 generate_output_gallery.py` scans `output/*/` and writes
`output/index.html`, a static gallery for eyeballing every generated run at once.

Both mondrian Python scripts are config-driven (`--config`, defaults to `configs/demo_v1_a4_pen.json`). **Always pass the same `--config` to both scripts in a run** — `generate_painting_paths.py`
reads the `painting_plan.json` that `mondrian_generator.py` wrote for that same config, and
mismatched configs silently mix profiles or fail to find the plan.

`painting_plan.json` and `painting_paths.json` are both intermediate representations in
millimeters (origin top-left, x right, y down) — **not** direct robot motor code. The ROS 2
adapter (`ros2/robross_painter`) is the only place that translates these into actual robot motion,
and robot calibration (taught paper corners, safe/contact Z height, tool offsets, home pose) lives
there (`config/demo_v1_rviz.yaml`), never inside the artwork/path configs. Preserve this
separation between artwork generation, path generation, validation, and robot execution.

### Route summaries

- **mondrian/**: Procedural vector design — `subdivide()` recursively splits the canvas,
  `generate_mondrian_layout()` styles per `palette_mode` (monochrome for Demo v1, color for
  legacy), `render_svg()` and `build_painting_plan()` consume the same geometry. Full architecture
  at `Image_Process/mondrian/README.md`.
- **sketch/**: Traces edges from a source bitmap — Canny edge detection + skeletonization +
  pixel-graph walk to centerlines → ordered `paint_stroke` segments. Full architecture at
  `Image_Process/sketch/README.md`.
- **image_to_mondrian/**: Classical (non-ML) color-quantization + connected-component pipeline —
  quantizes a photo to the robot's actual 5-color pen palette, segments same-color regions, fully
  fills each (not just outlines), plus black grid lines between color blocks. Can guarantee
  gap-free fill without vectorizing an ML model's output. Full architecture at
  `Image_Process/image_to_mondrian/README.md`.
- **gemini_mondrian/**: Gemini image-to-image restyle → its **own** simplified
  quantize/segment/fill/border-trace pipeline. `color_quantize.py`/`segmentation.py` are
  deliberately simpler than image_to_mondrian's (no bilateral filtering, no chroma gating,
  no face protection) because Gemini's output is already clean and flat-colored.
  `region_fill.py`/`border_tracing.py`/`path_ordering.py`/`path_validation.py` are copied
  byte-for-byte from image_to_mondrian (pure geometry, no real-photo assumptions). Full
  architecture at `Image_Process/gemini_mondrian/README.md`.

### Byte-for-byte copied modules

These modules are maintained as exact copies across route folders (see file-top comments for the
canonical source). The route self-contained architecture intentionally duplicates rather than
shares them — each route folder is a standalone pipeline that doesn't import from sibling folders:

| Module | Canonical source | Copies in |
|--------|-----------------|-----------|
| `path_validation.py` | `image_to_mondrian/` | `gemini_mondrian/`, `sketch/` |
| `path_ordering.py` | `image_to_mondrian/` | `gemini_mondrian/`, `sketch/` |
| `border_tracing.py` | `image_to_mondrian/` | `gemini_mondrian/` |
| `region_fill.py` | `image_to_mondrian/` | `gemini_mondrian/` |

When modifying any of these, update the canonical source first, then sync the copies.

### Key architectural constraints

- **sys.modules collision**: `config_loader`/`path_validation`/`path_ordering` are same-named
  top-level modules in each route folder. Importing two routes' modules into one long-lived Python
  process would collide in `sys.modules` — this is why `webapp/route_adapters.py` shells out via
  `subprocess.run()` instead of importing generation code.
- **MoveGroupInterface spinning**: In ROS 2 Humble, `MoveGroupInterface` spins the node internally.
  Do not spin it in an external executor — a second executor can steal its action responses.
- **Retiming start state**: The painting executor builds the retiming start state from the
  trajectory's first waypoint instead of calling `getCurrentState()` (which returns stale data
  while MoveIt's internal node is spinning).

## Commands

Run everything from the repo root (scripts use CWD-relative `configs/`/`output/` paths).

```bash
# Mondrian route — Demo v1 (A4, pen, monochrome)
python3 Image_Process/mondrian/mondrian_generator.py --config configs/demo_v1_a4_pen.json --seed 123
python3 Image_Process/mondrian/generate_painting_paths.py --config configs/demo_v1_a4_pen.json

# Sketch route — trace a source image's edges
python3 Image_Process/sketch/generate_sketch_paths.py --config configs/sketch_demo_a4.json

# image_to_mondrian route — quantize + fill a source photo
python3 Image_Process/image_to_mondrian/generate_painting_paths.py --config configs/image_to_mondrian_demo_a4.json

# gemini_mondrian route — Gemini restyle + vectorize (needs GEMINI_API_KEY env var)
python3 Image_Process/gemini_mondrian/generate_painting_paths.py --config configs/gemini_mondrian_demo_a4.json

# Test line — single 50mm first-contact stroke (docs/hardware-test-checklist.md section 9)
python3 Image_Process/mondrian/generate_test_line.py

# Test suites
python3 -m unittest discover Image_Process/mondrian/tests
python3 -m unittest discover Image_Process/image_to_mondrian/tests
python3 -m unittest discover Image_Process/gemini_mondrian/tests

# Regenerate the static output gallery
python3 generate_output_gallery.py
```

Same `--seed` + config produces deterministic output. Omit `--seed` for a random one (printed after
generation, so it can be copied back in to reproduce that exact layout).

The mondrian route is pure standard library. The sketch route needs `opencv-python` + `numpy` +
`scikit-image`. The image_to_mondrian route needs `opencv-python` + `numpy`. The gemini_mondrian
route needs `google-genai` (lazily imported, only in `gemini_client.py`). See each route's README
for install commands.

### ROS 2 side (`ros2/robross_painter`)

Built as a normal ROS 2 (Humble) package inside a colcon workspace alongside the RobRoss-maintained
Aubo driver fork (`ros2/robross_aubo.repos` is the vcstool manifest). Full workspace bootstrap and
three-terminal fake-hardware RViz flow is documented in `README.md` ("ROS 2 Aubo Setup") and
`ros2/robross_painter/README.md` — don't duplicate it here. Key points if touching this package:

- `painting_executor` (`src/painting_executor.cpp`) reads a `painting_paths.json`-format file and
  maps each command to MoveIt motion. See `ros2/robross_painter/README.md` for the command table.
- Safety checks refuse out-of-canvas coordinates, `move_to` with the pen down, and strokes with
  the pen up.

## Config profiles

Core profiles in `configs/`; use `demo_v1_a4_pen.json` unless exercising a different route or the
legacy behavior:

| Config | Canvas | Route | Notes |
| --- | --- | --- | --- |
| `demo_v1_a4_pen.json` | A4 210×297mm, 10mm margin | mondrian | monochrome (lines only), 1mm pen |
| `mondrian_12x12_paint.json` | 12in square, no margin | mondrian | red/yellow/blue accents, 10mm brush |
| `sketch_demo_a4.json` | A4 210×297mm, 20mm margin | sketch | traces `Image_Process/assets/apple.png` |
| `image_to_mondrian_demo_a4.json` | A4 210×297mm, 10mm margin | image_to_mondrian | quantizes+fills to 5 colors, 3mm pen |
| `gemini_mondrian_demo_a4.json` | A4 210×297mm, 10mm margin | gemini_mondrian | Gemini restyle + own vectorization |
| `gemini_mondrian_lenna_a4.json` | A4 210×297mm, 10mm margin | gemini_mondrian | Lenna test image variant |
| `gemini_mondrian_man_a4.json` | A4 210×297mm, 10mm margin | gemini_mondrian | Man photo variant (was minions) |
| `image_to_mondrian_lenna_a4.json` | A4 210×297mm, 10mm margin | image_to_mondrian | Lenna tuning config (bilateral, chroma) |
| `image_to_mondrian_minions_a4.json` | A4 210×297mm, 10mm margin | image_to_mondrian | Minions test variant |

Config field references live in each route's README.

## Repo layout

- `configs/` — pipeline config profiles (see above).
- `docs/` — `Rob_Ross_Prototype_v1.md` (current prototype source of truth),
  `painting-paths-format.md` (path JSON schema), `hardware-test-checklist.md`,
  `Rob_Ross_Discuss.md` (early brainstorming, not current requirements).
- `Image_Process/` — one subfolder per artwork-generation algorithm. New generation approaches
  get their own sibling subfolder rather than growing inside an existing one. See
  `Image_Process/README.md` for the module index.
- `Image_Process/assets/` — unified sample images (apple.png, lenna.png, sample.jpg, minions.jpg,
  images.jpeg). All config `source_image.path` fields point here.
- `output/` — generated artifacts (plans, paths, SVG previews), one subfolder per config profile.
  Intentionally committed (seed-123 reference samples), not gitignored.
- `ros2/` — `robross_painter` ROS 2 package (MoveIt executor) and the vcstool manifest for the
  Aubo driver fork workspace.
- `webapp/` — optional local Flask control panel: upload a photo, pick a route, click Process.
  Drives each route's CLI scripts as subprocesses (never imports route modules into the same
  process — see Key architectural constraints above). Route-specific details live in one dict entry
  per route in `route_adapters.py`.
- `generate_output_gallery.py` — repo-root, stdlib-only script that scans `output/*/` and writes
  `output/index.html`, a static gallery (previews + `debug` stats + validation status per run).
  Rerun after regenerating any config's output; not wired into the generation scripts themselves
  since not every run (e.g. CI validation) needs the extra HTML artifact.
- `CAD/` — SolidWorks 2025 SP5.0 prototype files for hardware (canvas/paint holders).
- `firmware/` — ESP32 servo gripper firmware (Arduino IDE sketch, continuous-rotation MG996R).
  See `firmware/gripper_esp32/README.md`.

## Conventions

- Prefer simple, readable Python using the standard library unless a dependency is clearly
  justified — `mondrian/` has zero third-party dependencies. Route-specific dependencies are
  deliberate, folder-scoped exceptions (see each route's README). Don't let dependencies spread to
  other folders without similarly clear justification.
- `webapp/`'s `flask` dependency is optional, scoped to that folder only. It shells out to each
  route's existing CLI, never imports generation code.
- `gemini_mondrian/gemini_client.py`'s `google-genai` dependency is imported lazily inside the
  single function that calls the Gemini API. The API key comes from the `GEMINI_API_KEY`
  environment variable only — never a config field or committed anywhere.
- `image_to_mondrian/face_protection.py`'s `mediapipe` dependency is optional (default off),
  imported lazily inside the one function that needs it. Don't reach for an ML dependency elsewhere
  in this repo without confirming the classical approach has a real ceiling.
- Update the relevant Markdown (`README.md`, route READMEs, `docs/painting-paths-format.md`)
  whenever behavior or project decisions change; these docs are living references, not one-off notes.
- Importing in tests: each route's `tests/context.py` adds the parent module directory to
  `sys.path` (scripts import each other as top-level modules, not a package). Import it first in
  new test files following the existing pattern.
