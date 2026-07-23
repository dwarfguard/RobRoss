# line_art

Image-to-robot-path pipeline for the **line_art / clean line-art tracing
route**: turns an already-clean line-art image — a technical illustration,
product diagram, or logo, not a photo — into a `painting_paths.json`-format
file. It exists alongside `sketch/` (which traces edges from photos and
other high-contrast bitmaps via Canny) because Canny is the wrong tool for
input that is already a flat black-line-on-white drawing: see "Why not
Canny?" below.

**Only this route needs third-party dependencies** — `opencv-python`,
`numpy`, and `scikit-image`, the same three as `sketch/`. `mondrian/` stays
pure standard library; this is a deliberate, folder-scoped exception (see
root `CLAUDE.md`). On this project's Ubuntu 22.04 / ROS 2 Humble setup, all
three are plain apt packages:

```bash
sudo apt-get install -y python3-opencv python3-numpy python3-skimage
```

## Why not Canny?

Canny edge detection finds gradient transitions. A drawn stroke with real
width (not a 1px hairline) has *two* transitions — white-to-black where it
starts, black-to-white where it ends. Canny's hysteresis thresholding finds
both, and skeletonizing that edge map collapses each side to its own
centerline: two parallel traced lines offset by roughly half the stroke
width, instead of one line down the middle. That's the "double line"
artifact you'd get if this route reused `sketch/canny_edges.py` directly on
a diagram like `Image_Process/assets/hinton.png`.

`line_art/` instead thresholds the grayscale image straight into a filled
stroke mask (`line_tracing.binarize()`) and skeletonizes *that*. There's
only one filled region per stroke, so it converges to one true centerline,
not two edge curves. The pixel-graph walk that turns a skeleton into
ordered centerline chains (`line_tracing.extract_strokes()`) is ported
unchanged from `sketch/canny_edges.py` — it was already decoupled from
Canny, treating its input as an opaque boolean skeleton array, so the same
walk works on a threshold-built skeleton too.

Threshold+skeletonize is also more prone to short spurious branches at
junctions and line-cap corners than skeletonizing a thin Canny edge map,
so this route adds a pruning step (`line_tracing.prune_spurs()`) that
`sketch/` doesn't need.

**Tune `binary_threshold` higher than it looks like you should.** An
antialiased raster export spreads a line's "ink" across several pixels of
increasingly light gray as it fades into the white background — those
lighter pixels are still part of the line, not noise. A too-low threshold
(this route originally shipped with 128, which turned out to be much too
strict for `hinton.png`) only catches each stroke's darkest core pixels,
which for a thin or diagonal line can be so sparse that the "line" flickers
above and below the cutoff along its own length — fragmenting one visual
line into dozens of disconnected mask blobs, which shows up downstream as
choppy/broken traced lines and, for small text, strokes disappearing
entirely. Checking `cv2.connectedComponents()` on the raw mask is a quick
sanity check: a clean diagram should have on the order of tens to low
hundreds of components, not thousands.

## Pipeline

```
Config profile (configs/*.json)
  -> line_tracing.binarize()             threshold grayscale -> boolean stroke mask
  -> line_tracing.close_mask()           morphological closing -> bridge antialiasing gaps
  -> line_tracing.skeletonize_mask()     mask -> single-pixel-wide skeleton
  -> line_tracing.prune_skeleton_spurs() structural pixel-level spur removal (see below)
  -> line_tracing.extract_strokes()      skeleton -> traced centerline chains (node-clustered)
  -> line_tracing.prune_spurs()          drop remaining short spurs/degenerate loops
  -> line_tracing.simplify()             Douglas-Peucker point reduction
  -> path_ordering.order_strokes()       greedy nearest-neighbor travel order
  -> generate_line_art_paths.py          -> output/<config-name>/<painting_paths_file> (+ preview SVG)
```

Like `sketch/`, there is no intermediate `painting_plan.json` step — no
vector design to describe first, so `generate_line_art_paths.py` goes
straight from traced pixel strokes to the final command list.

### Usage

```bash
python3 Image_Process/line_art/generate_line_art_paths.py --config configs/line_art_demo_a4.json
```

`--config` defaults to `configs/line_art_demo_a4.json`, which traces
`Image_Process/assets/hinton.png`. Run from the repo root (paths in the
config are CWD-relative), same as the other routes' scripts.

### Config fields

| Field | Meaning |
| --- | --- |
| `canvas.width_mm` / `height_mm` / `margin_mm` / `origin` | Same meaning as the other routes' config schema — `origin` must be `"top-left"`, `margin_mm` keeps the traced drawing inside the paper edge. |
| `source_image.path` | Path to the source line-art bitmap. RGBA is composited onto white using its alpha channel before thresholding, so both opaque exports and transparent-background art work the same way. |
| `source_image.binary_threshold` | Grayscale cutoff (0-255, default 200): pixels darker than this are treated as part of a drawn line. See "Why not Canny?" above for why this needs to be higher than it looks like it should for antialiased raster art. |
| `source_image.morph_close_kernel_px` | Morphological closing kernel size (pixels, default 2; 0 disables) applied to the mask before skeletonizing, to bridge small antialiasing-induced gaps. |
| `source_image.min_spur_length_px` | Length threshold (pixels) used at two stages: structural spur removal on the raw skeleton before tracing, and a second pass dropping any short open branch or degenerate closed loop that survives extraction — see `prune_skeleton_spurs`/`prune_spurs` below. Default 4.0; a font-like image with sharp corners (e.g. `ai.png`) may need it higher (~7-8) to fully close ring-shaped letterforms — see "Known limitations". |
| `source_image.min_stroke_length_mm` | After mapping to canvas millimeters, drop any stroke shorter than this — filters sub-pixel noise that survives spur pruning. Default 1.0. |
| `source_image.simplify_epsilon_ratio` | Douglas-Peucker simplification strength as a fraction of each stroke's arc length — larger = fewer points/straighter lines. Default 0.002. |
| `path_generation.tool_width_mm` | Pen stroke width (preview rendering + `path_settings.tool_width_mm` in the output). |
| `path_generation.home_position_mm` | Where the pen starts before the first stroke; affects greedy-ordering's first pick. Optional, defaults to the canvas's top-left margin corner. |
| `output.directory`, `.painting_paths_file`, `.preview_svg_file` | Same meaning as the other routes' config schema. |

## line_tracing.py

`load_grayscale(image_path)` / `binarize(image_path, threshold)`: reads the
image (compositing RGBA onto white via its alpha channel first, if
present), thresholds it into a boolean stroke mask. `close_mask(mask,
kernel_px)`: morphological closing to bridge small gaps left by
antialiasing before skeletonizing (`kernel_px <= 0` is a no-op).
`skeletonize_mask()` thins the (closed) mask to single-pixel centerlines
via `skimage.morphology.skeletonize`.

`prune_skeleton_spurs(skeleton, min_length_px)`: removes short dead-end
branches directly from the skeleton's pixel set, *before* the pixel graph
is built — not after. A sharp corner (two strokes meeting at a point) makes
`skeletonize()` grow a short "whisker" off that point; if that whisker were
only cleaned up after the fact (by filtering already-extracted strokes),
the damage is already done — the corner pixel was counted as a degree≥3
junction while the whisker existed, so a closed loop that should pass
straight through that corner gets cut into separate open fragments there.
Removing the whisker first lets the corner's degree fall back to 2, so
`extract_strokes()` treats it as an ordinary pass-through pixel and the
loop stays whole. Runs iteratively to a fixed point, since removing one
spur can expose a new one at a neighboring junction.

`extract_strokes(skeleton)`: walks the skeleton's pixel graph (endpoints =
degree 1, junctions = degree ≥ 3, treated as graph nodes; runs of degree-2
pixels between them as edges, each walked once) to produce one entry per
independent line — pixel `(x, y)` points plus whether it's a closed loop.
Ported from `sketch/canny_edges.py`'s tracer, with one addition: junction
pixels that are 8-adjacent to each other are clustered into a single
logical vertex (a rounded corner often rasterizes into 2-3 node pixels
sitting right next to each other instead of one). Intra-cluster hops are
skipped, and a stroke whose two ends land in the same cluster counts as
closed — not just a stroke that returns to its exact starting pixel.
Without this, those extra adjacent node pixels spawn near-duplicate paths
and near-zero-length spurious edges that fragment what should be one
continuous stroke.

`prune_spurs(strokes, min_length_px)`: a second, coarser pass after
extraction — drops any *open* stroke shorter than `min_length_px` (spurs
`prune_skeleton_spurs` didn't need to touch, e.g. off a junction that
stayed valid), and any *closed* loop shorter than `min_length_px` too.
Short closed loops this size are degenerate pixel-level noise left over at
a clustered-node boundary, not real small content — real closed shapes
(a dot, a letter's counter) are comfortably longer than any reasonable
spur threshold, so this doesn't risk real content.

`simplify(points_xy, closed, epsilon_ratio)`: Douglas-Peucker point
reduction (`cv2.approxPolyDP`), same as `sketch/canny_edges.py`'s.

## path_ordering.py / path_validation.py

Copied byte-for-byte from `Image_Process/image_to_mondrian/` (the
canonical source — see each file's top comment), same as `sketch/`'s and
`gemini_mondrian/`'s copies. `path_validation.py`'s `paint_path` branch
(full `points_mm` / bounds / total-length checks, not just an "unknown
command" warning) was added to the canonical source specifically to
support this route, since `line_art/` emits `paint_path` rather than
`paint_stroke` (see `generate_line_art_paths.py` below) — the other two
copies picked up the same addition when synced.

`line_art/` doesn't need `border_tracing.py` or `region_fill.py` — those
are for routes that fill closed regions with color; this route only traces
open/closed lines, it never fills.

## generate_line_art_paths.py

`build_canvas_strokes()` runs the full trace-and-simplify pipeline on the
config's source image, then scales pixel points onto the canvas's drawable
box (canvas size minus `margin_mm` on each side), preserving aspect ratio
and centering the result, then filters by `min_stroke_length_mm`.

For closed strokes, it also re-appends the first (simplified) point to the
end of the point list right after `simplify()`. `simplify()`'s
Douglas-Peucker (`cv2.approxPolyDP(..., closed=True)`) returns a closed
loop as a bare polygon — vertices only, no repeated closing point — and
nothing downstream (`path_ordering.order_strokes()`, `paint_path`) tracks
"closed" at all once the stroke's own point list is the only thing passed
along. Without explicitly closing the loop here, the pen would lift one
edge short of a full circuit even for a stroke `extract_strokes()`
correctly identified as closed — this is what left `ai.png`'s letterform
outlines with a small notch even after the node-clustering fix above.

`main()` orders the strokes, then `build_commands()` turns each ordered
stroke into one `move_to -> lower_tool -> paint_path -> lift_tool`
sequence — **one continuous `paint_path` per traced line**, not one
`paint_stroke` per point pair like `sketch/`: a diagram's outline or a
hatching mark is one continuous pen-down motion, matching how
`Image_Process/mondrian/generate_painting_paths.py` treats curved/long
lines (see `docs/painting-paths-format.md`). A single
`select_tool`/`dip_paint` (black) is emitted once up front, since this
route only ever produces one monochrome pen tool.

Before writing output, runs `path_validation.validate_painting_paths()`
and stores the result under `painting_paths["validation"]`; exits non-zero
on validation errors, same convention as every other route.

## assets/hinton.png

Reference source image (`Image_Process/assets/hinton.png`): a clean
black-line technical illustration of a robot arm on a wheeled base, RGBA,
antialiased strokes, pure white background — the profile this route is
tuned for. To trace a different image, point `source_image.path` at it and
adjust `binary_threshold` if the new image's line/background contrast
differs.

## Known limitations

Very small text (letterforms only 1-2px stroke-wide at the source image's
resolution, e.g. `hinton.png`'s circular "HintonAI" badge) can still come
out with adjacent letters partially fused into a squiggle after
skeletonizing, even with a well-tuned threshold and closing kernel — the
individual strokes are close enough together that there just aren't enough
pixels between them to keep them topologically separate. This is a raster
resolution limit, not a parameter to tune away; if legible text tracing
matters, increase the source image's resolution (re-export the diagram
larger) rather than fighting it with `binary_threshold`/
`morph_close_kernel_px`.

**Outline/bubble fonts need a higher `min_spur_length_px`.** A source image
where every glyph is drawn as a hollow ring (constant-width outline stroke,
e.g. `configs/line_art_ai_a4.json`'s `ai.png`) has letterforms that should
each skeletonize to one clean closed loop. But a sharp corner (like the top
of an "A") makes the loop pinch to near-zero local width, which
`skeletonize()` turns into a short spur; the default `min_spur_length_px`
(4.0, tuned for `hinton.png`'s hatching-heavy content) can be too low to
fully remove that spur, leaving the loop split into open fragments even
after node clustering. Check with `cv2.connectedComponents()` /
`cv2.findContours(..., cv2.RETR_CCOMP)` on the binarized mask if this
happens: an outline font's contour count (outer + inner per glyph) should
roughly match `extract_strokes()`'s closed-loop count once traced; if it
doesn't, raise `min_spur_length_px` a few pixels at a time until it does.
