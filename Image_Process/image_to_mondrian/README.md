# image_to_mondrian

A third artwork-generation route, alongside `Image_Process/mondrian/` (random
procedural vector layouts) and `Image_Process/sketch/` (Canny outline
tracing, lines only). This route takes an arbitrary source **photo**,
quantizes it down to the robot's actual pen colors (red/blue/yellow/black/
white by default), segments it into connected same-color regions, and
**fully fills** each region — not just outlines — plus classic Mondrian
black grid lines between color blocks. Output is the same
`painting_paths.json` command format the other two routes produce, so it's
consumable by the same `ros2/robross_painter` executor.

```text
Config profile (configs/*.json)
  -> color_quantize.quantize_to_palette()    photo -> 5-color label image
  -> segmentation.segment_image()            per-color connected regions, noise-filtered
  -> region_fill.region_to_pixel_strokes()   erode + scanline fill, per region
  -> border_tracing.trace_boundary_strokes() black grid lines between color blocks
  -> path_ordering.order_strokes()           greedy nearest-neighbor travel order, per color group
  -> generate_painting_paths.py              -> output/<painting_paths_file> (+ preview SVG + quantized preview PNG)
```

There is no intermediate `painting_plan.json` step (like `sketch/`, not like
`mondrian/`) — there's no vector design to describe before rasterizing a
photo, so the pipeline goes straight to `painting_paths.json`.

## Why non-ML

This route was deliberately built as a classical color-quantization +
connected-component pipeline rather than an ML segmentation/GAN/diffusion
approach. The hard requirement is **complete, gap-free fill** — a matrix
that classical scanline fill on axis-independent region masks satisfies
directly, while ML approaches (position-based segmentation models, GAN/
diffusion image generation) all eventually produce a raster result that
still needs vectorizing into fillable regions, which is where the real risk
and complexity lives. A future ML segmentation model (e.g. SAM) could
replace just the `segmentation.py` step without touching anything
downstream, if photo quantization + connected-components proves too coarse
for some inputs.

## Usage

```bash
# Needs opencv-python + numpy + scikit-image - see "Dependencies" below
python3 Image_Process/image_to_mondrian/generate_painting_paths.py --config configs/image_to_mondrian_demo_a4.json
```

`--config` defaults to `configs/image_to_mondrian_demo_a4.json`. Run from
the repo root — `source_image.path`, `output.directory`, etc. are all
CWD-relative.

Outputs (all under `output/` per the config):

- `image_to_mondrian_painting_paths.json` — the command list a robot adapter consumes.
- `image_to_mondrian_path_preview.svg` — static visual preview of the strokes.
- `image_to_mondrian_quantized_preview.png` — debug image: every pixel painted
  its quantized palette color (before segmentation/small-region filtering),
  useful for tuning `min_region_area_mm2`/`palette`/blur settings without a
  full run.

## Dependencies

Same three third-party packages as `Image_Process/sketch/` (this repo's
other photo-input route) — `opencv-python`, `numpy`, `scikit-image` (the
last one for `skimage.morphology.skeletonize`, used by the black-grid-line
tracer). `Image_Process/mondrian/` stays pure standard library; this is a
second, deliberate folder-scoped dependency exception per root `CLAUDE.md`.

```bash
sudo apt-get install -y python3-opencv python3-numpy python3-skimage
```

## Config fields (`configs/image_to_mondrian_demo_a4.json`)

| Section.field | Meaning |
| --- | --- |
| `canvas.*` | Same meaning/validation as the other two routes (`width_mm`/`height_mm`/`margin_mm`/`origin`). |
| `source_image.path` | Path to the input photo. |
| `source_image.blur_kernel_size` / `blur_sigma` | Gaussian blur applied before quantization, to merge single-pixel noise so it doesn't fragment into tiny regions. |
| `source_image.downscale_max_dimension_px` | Caps the image's longest side before processing (keeps region counts and runtime tractable). |
| `palette.colors` | The list of `{name, hex}` target colors — configurable, but defaults to the robot's actual 5 pens (white/yellow/red/blue/black). Order is also the pen-switching order. |
| `palette.color_space` | `"lab"` (recommended, perceptually uniform nearest-color matching) or `"rgb"`. |
| `segmentation.min_region_area_mm2` | Regions smaller than this are dropped (left unpainted) rather than filled — noise/detail below the pen's practical resolution. |
| `segmentation.morph_open_kernel_px` | Morphological opening kernel size, strips single-pixel speckle from each color's mask before labeling *and* before border tracing. |
| `segmentation.skip_white_regions` | If true (default), white-quantized regions get zero paint strokes — assumes white paper, no white pen needed. Set false if painting on non-white material with an actual white pen. |
| `path_generation.tool_width_mm` / `stroke_overlap_ratio` | Same meaning as the other two routes' stripe-fill spacing. |
| `path_generation.mask_erosion_mm` | This route's analog of `mondrian`'s `edge_inset_mm`, but for an arbitrary-shape mask instead of a rectangle — shrinks each region's paintable area inward so the tool's width doesn't bleed into a neighboring color. |
| `path_generation.home_position_mm` | Starting position for travel-distance ordering; defaults to the canvas' top-left margin corner. |
| `border_generation.draw_borders` | Whether to trace and paint the classic Mondrian black grid lines between color blocks. |
| `border_generation.simplify_epsilon_ratio` | Douglas-Peucker simplification strength for the traced border lines (same idea as `sketch/`'s field of the same name). |
| `output.quantized_preview_png_file` | Optional debug PNG (see above); omit to skip writing it. |

## Modules

- **`color_quantize.py`** — `load_image()`/`preprocess()` (resize + Gaussian
  blur) /`quantize_to_palette()` (fully vectorized nearest-palette-color
  matching in Lab or RGB space, no per-pixel Python loop).
- **`segmentation.py`** — `clean_label_image()` (per-color morphological
  open, strips speckle — pixels no color claims after opening are `-1`,
  distinct from any real palette index), `label_connected_regions()`
  (`cv2.connectedComponentsWithStats` per color), `filter_small_regions()`,
  `segment_image()` composes all three. Returns the *cleaned* (speckle-
  opened but not small-region-filtered) label image alongside the kept
  regions — that cleaned image is also the correct input for border tracing
  (see below), not the raw per-pixel quantization.
- **`region_fill.py`** — the new polygon-fill algorithm: `erode_mask()`
  shrinks a region inward by `mask_erosion_mm` so strokes don't bleed
  across color boundaries; `find_row_intervals()` is a vectorized run-length
  detector that finds every contiguous paintable interval in one mask row
  (handles concave shapes with more than one interval per row);
  `compute_stripe_rows()` mirrors `mondrian`'s `compute_stripe_row_centers()`
  spacing formula in pixel-row space; `region_to_pixel_strokes()` composes
  all three into one region's fill strokes. No manual left-right
  alternation here (unlike `mondrian`'s rectangle boustrophedon) — a
  concave shape's per-row interval count varies, so travel-order
  optimization is deferred entirely to `path_ordering.order_strokes()`.
- **`border_tracing.py`** — `compute_boundary_mask()` finds every pixel that
  differs from its up/left neighbor in the *cleaned* label image, in one
  pass — this naturally avoids drawing a shared edge between two regions
  twice (once per side). `trace_boundary_strokes()` is copied from
  `sketch/canny_edges.py`'s skeleton pixel-graph walk, generalized to accept
  any boolean boundary mask instead of always running `cv2.Canny()`
  internally. `simplify()` is the same Douglas-Peucker copy `sketch/` uses.
- **`generate_painting_paths.py`** — orchestrates the whole pipeline;
  `image_to_canvas_transform()`/`px_to_mm()` is the same aspect-fit-and-
  center mapping `sketch/generate_sketch_paths.py::map_points_to_canvas()`
  uses, copied and split into a reusable transform. `order_and_build_commands()`
  groups strokes by color first (physical pen changes are the expensive
  part, not travel distance), greedy-orders each color group with
  `path_ordering.order_strokes()`, then draws the black grid lines **last**
  — same convention `mondrian` uses for its border, and it has a nice side
  effect here too: `mask_erosion_mm` leaves a thin gap between adjacent
  fills, and the border line (traced along the true, un-eroded boundary)
  paints right over that gap.

## Known v1 limitations

- Small regions dropped by `min_region_area_mm2` are left unpainted rather
  than merged into a neighboring region — a documented simplification, not
  a bug.
- Border tracing runs on the *cleaned* (speckle-opened) label image, not the
  *filtered* one — so a region that survives opening but still gets dropped
  by the small-region-area filter will still get a black outline drawn
  around its (unpainted) footprint. Rare in practice since both filters use
  similar-scale thresholds, but possible.
