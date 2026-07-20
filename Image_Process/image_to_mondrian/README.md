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
  -> color_quantize.preprocess()             resize + Gaussian blur or bilateral filter
  -> face_protection.detect_protected_face_mask()  optional: eye/eyebrow/lip landmarks to protect
  -> color_quantize.quantize_to_palette()    photo -> 5-color label image
  -> segmentation.segment_image()            per-color connected regions, noise-filtered, protected pixels preserved
  -> region_fill.region_to_pixel_strokes()   erode + scanline fill, per region
  -> border_tracing.trace_region_contours()  black grid lines between color blocks
  -> path_ordering.order_strokes()           greedy nearest-neighbor travel order, per color group
  -> generate_painting_paths.py              -> output/<painting_paths_file> (+ preview SVG + quantized preview PNG)
```

There is no intermediate `painting_plan.json` step (like `sketch/`, not like
`mondrian/`) — there's no vector design to describe before rasterizing a
photo, so the pipeline goes straight to `painting_paths.json`.

## Why (mostly) non-ML

This route is built as a classical color-quantization + connected-component
pipeline rather than an ML segmentation/GAN/diffusion approach. The hard
requirement is **complete, gap-free fill** — a matrix that classical
scanline fill on axis-independent region masks satisfies directly, while
GAN/diffusion image generation all eventually produce a raster result that
still needs vectorizing into fillable regions, which is where the real risk
and complexity lives.

One optional, narrowly-scoped exception: `face_protection.py` uses a
pretrained (no training, CPU inference) MediaPipe Face Mesh model to find
eye/eyebrow/lip landmark regions in photos of faces. This exists because
classical morphological closing (see `morph_close_kernel_px` below) turned
out to have a real ceiling — a closing kernel large enough to bridge a gap
cut into one large object by lighting/shadow (e.g. a hat's shadowed brim
reading as a different color than its lit crown) is *also* large enough to
merge small nearby features like an eye into the surrounding hair, because
closing only sees pixel geometry, not "this is an eye" — verified this
isn't a tuning problem: an eye's own pixel blob is often already connected
to nearby hair through eyebrow/eyelash strands *before* any morphology
runs at all, so no kernel size threads that needle. A real segmentation
signal is the direct fix for that specific failure mode; a GAN wouldn't
help here (GANs generate images, they don't provide the "this is an eye"
labels this problem actually needs). This is opt-in
(`segmentation.protect_face_features`) and the rest of the pipeline
(including the fill/border/travel-ordering algorithms) doesn't use ML at
all — a future general segmentation model (e.g. SAM) could still replace
just the `segmentation.py` labeling step for non-face subjects, if photo
quantization + connected-components proves too coarse.

## Usage

```bash
# Needs opencv-python + numpy - see "Dependencies" below
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

`opencv-python` + `numpy` — used for image loading/quantization, connected-
component segmentation, scanline fill, and (`cv2.findContours`) the
black-grid-line tracer. Unlike `Image_Process/sketch/` (this repo's other
photo-input route), no `scikit-image` is needed here — border tracing is
per-region contour extraction, not skeleton/graph-walk based.
`Image_Process/mondrian/` stays pure standard library; this is a second,
deliberate folder-scoped dependency exception per root `CLAUDE.md`.

```bash
sudo apt-get install -y python3-opencv python3-numpy
```

**Optional**: `segmentation.protect_face_features: true` needs `mediapipe`
(~30-50MB, bundles its own TFLite runtime and pretrained model — no
separate model download, no training, no GPU required):

```bash
pip install mediapipe
```

Everything else works with `protect_face_features` left unset/`false` even
if `mediapipe` isn't installed — `face_protection.py` only imports it
inside the function that needs it, so the rest of the module has no hard
dependency on it.

## Config fields (`configs/image_to_mondrian_demo_a4.json`)

| Section.field | Meaning |
| --- | --- |
| `canvas.*` | Same meaning/validation as the other two routes (`width_mm`/`height_mm`/`margin_mm`/`origin`). |
| `source_image.path` | Path to the input photo. |
| `source_image.blur_kernel_size` / `blur_sigma` | Gaussian blur applied before quantization, to merge single-pixel noise so it doesn't fragment into tiny regions. Ignored when `bilateral_d` is set (see below). |
| `source_image.bilateral_d` / `bilateral_sigma_color` / `bilateral_sigma_space` | Optional, `bilateral_d` default `0` (disabled). Edge-preserving smoothing (`cv2.bilateralFilter`) instead of Gaussian blur — smooths flat/gradient areas (e.g. a lit-to-shadow transition across one object) while keeping real high-contrast edges (e.g. an eye against skin) sharp, which Gaussian blur can't distinguish. Recommended over `blur_kernel_size` for real photos with faces or other fine detail worth preserving; `sigma_color`/`sigma_space` default to 75/75, the standard general-purpose starting point in the literature, not tuned to any one photo. |
| `source_image.downscale_max_dimension_px` | Caps the image's longest side before processing (keeps region counts and runtime tractable). |
| `palette.colors` | The list of `{name, hex}` target colors — configurable, but defaults to the robot's actual 5 pens (white/yellow/red/blue/black). Order is also the pen-switching order. |
| `palette.color_space` | `"lab"` (recommended, perceptually uniform nearest-color matching) or `"rgb"`. |
| `palette.neutral_chroma_threshold` | Optional, Lab only, default `0` (disabled). Pixels with Lab chroma below this are only matched against the palette's neutral colors (white/black), never a chromatic one — keeps desaturated/mid-tone content (skin, hair, shadow) from being forced into a bold color just because it's marginally closer than white, so photos come out closer to real Mondrian paintings (mostly white canvas, sparingly-used bold color blocks) instead of every pixel being colored in. A fixed number tuned for one specific photo, not a general default — prefer `neutral_chroma_percentile` below unless you're deliberately overriding it. |
| `palette.neutral_chroma_percentile` | Optional (0-100). Computes `neutral_chroma_threshold` automatically as this percentile of *this photo's own* Lab chroma distribution (see `color_quantize.compute_adaptive_chroma_threshold()`), instead of a fixed number picked for one photo — the threshold self-adjusts to how colorful the source image actually is. Ignored if `neutral_chroma_threshold` is also set explicitly (explicit wins). Validated against three visually different photos (a real portrait, a real product photo, a flat synthetic cartoon) — all three produced sensible thresholds with the same percentile value, no per-photo tuning. |
| `segmentation.min_region_area_mm2` | Regions smaller than this are dropped (left unpainted) rather than filled — noise/detail below the pen's practical resolution. |
| `segmentation.morph_open_kernel_px` | Morphological opening kernel size, strips single-pixel speckle from each color's mask before labeling *and* before border tracing. |
| `segmentation.morph_close_kernel_px` | Optional, default `0` (disabled). Morphological closing kernel size, run *before* opening — bridges gaps cut into one real object's mask by internal lighting/shadow variation (e.g. a hat's shadowed brim reading as a different, more saturated color than its lit crown, splitting one object into a fragmented, unrecognizable shape). Needs tuning per photo relative to `downscale_max_dimension_px` (too small doesn't bridge the gap; too large can merge separate same-colored regions that happen to be near each other into one oversized blob, **and** can merge small nearby semantic features like an eye into hair — see `protect_face_features` below for the fix) — see `configs/image_to_mondrian_lenna_a4.json` for a worked example. Pair with a smaller `border_generation.simplify_epsilon_ratio` than you'd otherwise use — the cleaner shapes closing produces are still curved, and an epsilon tuned for fragmented jagged contours will over-simplify a real curve back into a jagged polygon. |
| `segmentation.skip_white_regions` | If true (default), white-quantized regions get zero paint strokes — assumes white paper, no white pen needed. Set false if painting on non-white material with an actual white pen. |
| `segmentation.protect_face_features` | Optional, default `false`. Requires `mediapipe` (see "Dependencies"). Runs a pretrained face landmark model on the source photo and locks eye/eyebrow/lip pixels to their original pre-morphology classification, no matter what `morph_close_kernel_px` decides for those pixels — lets you use a closing kernel large enough to properly fix a large object's shape without that same kernel erasing nearby facial detail. A photo with no detected face (or `mediapipe` not installed) silently no-ops — not every photo has a face, that's not an error. |
| `segmentation.face_protection_margin_px` | Optional, default `0`. Extra dilation (in pixels, at the processed/downscaled resolution) added around each detected landmark region, in case the landmark polygon alone runs slightly tighter than the visually-relevant feature. |
| `path_generation.tool_width_mm` / `stroke_overlap_ratio` | Same meaning as the other two routes' stripe-fill spacing. |
| `path_generation.mask_erosion_mm` | This route's analog of `mondrian`'s `edge_inset_mm`, but for an arbitrary-shape mask instead of a rectangle — shrinks each region's paintable area inward so the tool's width doesn't bleed into a neighboring color. |
| `path_generation.home_position_mm` | Starting position for travel-distance ordering; defaults to the canvas' top-left margin corner. |
| `border_generation.draw_borders` | Whether to trace and paint the classic Mondrian black grid lines between color blocks. |
| `border_generation.simplify_epsilon_ratio` | Douglas-Peucker simplification strength for the traced border lines (same idea as `sketch/`'s field of the same name). |
| `output.quantized_preview_png_file` | Optional debug PNG (see above); omit to skip writing it. |

## Modules

- **`color_quantize.py`** — `load_image()`/`preprocess()` (resize + Gaussian
  blur *or* edge-preserving bilateral filter — see `bilateral_d` above) /
  `quantize_to_palette()` (fully vectorized nearest-palette-color matching
  in Lab or RGB space, no per-pixel Python loop, with the optional
  `neutral_chroma_threshold` gating described above) /
  `compute_adaptive_chroma_threshold()` (Nth percentile of the photo's own
  Lab chroma distribution — backs `palette.neutral_chroma_percentile`).
- **`segmentation.py`** — `clean_label_image()` runs per-color morphological
  closing (optional, bridges lighting/shadow-induced gaps within one real
  object's mask — see `morph_close_kernel_px` above) *then* opening
  (strips speckle — closing first so opening doesn't erase a gap's ragged
  edge pixels before closing gets a chance to bridge it), then — if a
  `protected_mask` was supplied — force-restores every protected pixel back
  to its pre-morphology classification, overriding whatever any color's
  close/open decided, regardless of processing order. Pixels no color
  claims afterward are `-1`, distinct from any real palette index.
  `label_connected_regions()` (`cv2.connectedComponentsWithStats` per
  color), `filter_small_regions()`, `segment_image()` composes all three.
  Each kept region dict carries its own boolean `mask` — that per-region
  mask is what both `region_fill.py` (fill) and `border_tracing.py`
  (outline) consume directly.
- **`face_protection.py`** — optional, `mediapipe`-dependent (see
  "Dependencies"). `detect_protected_face_mask()` runs MediaPipe Face Mesh
  on the source photo and fills each detected face's eye/eyebrow/lip
  landmark groups as *separate* convex hulls (combining every group's
  points into one shared hull first would fuse the left eye, right eye, and
  lips into a single blob, defeating the point of tracking them
  separately), dilated by `face_protection_margin_px`. Returns `None` (not
  an error) when `mediapipe` isn't installed or no face is detected — feeds
  directly into `segmentation.segment_image()`'s `protected_mask` param.
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
- **`border_tracing.py`** — `trace_region_contours()` runs `cv2.findContours()`
  on one region's own mask at a time (one closed loop per outer boundary,
  one per hole), instead of walking a single boundary-pixel network across
  the whole image. This matters for busy real photos: a global boundary
  network has huge numbers of junctions (every point where 3+ quantized
  colors meet), and any graph walk has to break a new stroke at every one —
  for a textured photo that fragments into thousands of tiny disconnected
  pieces (each one costs the robot a full lift/travel/lower cycle). A single
  region's own boundary is always just one or a few closed loops no matter
  how jagged the pixel boundary is, so per-region contour tracing sidesteps
  the fragmentation entirely. Trade-off: a shared edge between two adjacent
  regions gets traced (and painted) once from each side, not deduplicated —
  see "Known v1 limitations". `simplify()` is the same Douglas-Peucker
  (`cv2.approxPolyDP`) helper `sketch/` uses.
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
- A shared edge between two adjacent kept regions gets traced (and painted)
  once from each region's own contour — not deduplicated. Visually this
  just makes that shared line marginally bolder; it doesn't affect fill
  quality. De-duplicating would need geometric edge-matching with a
  tolerance, and the win from switching to per-region contour tracing
  (thousands of fragments down to roughly one-to-a-few strokes per region)
  is large enough that this wasn't worth the added complexity.
- `protect_face_features` only protects eye/eyebrow/lip landmark regions,
  not a full face outline — a large `morph_close_kernel_px` can still merge
  other nearby content (e.g. two unrelated dark areas of the background)
  into one region; this is expected morphological behavior, not a bug, and
  is a separate concern from face detail preservation. Face detection
  itself is MediaPipe Face Mesh's own scope: works best on a clear, mostly
  frontal face; a photo with no detectable face just gets no protection
  (silently, not an error).
