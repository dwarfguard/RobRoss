# sketch

Image-to-robot-path pipeline for the **sketch route**. It builds one
monochrome robot job in two ordered phases: long structural outlines first,
then a bounded set of identity-defining details from the original photo.
Optional Qwen preprocessing makes overlapping dark forms easier to read;
the route remains fully usable offline through deterministic local fallback.

This route needs `opencv-python`, `numpy`, `scikit-image`, and Pillow for Canny
edge detection, skeletonization, and EXIF orientation normalization. Qwen enhancement additionally uses `requests`
and a Beijing-region `DASHSCOPE_API_KEY`. See
`Image_Process/README.md` for the dependency summary. On this project's Ubuntu
22.04 / ROS 2 Humble setup, all three are plain apt packages:

```bash
sudo apt-get install -y python3-opencv python3-numpy python3-skimage python3-pil
# Semantic outline enhancement:
python3 -m pip install requests
export DASHSCOPE_API_KEY="your-Beijing-Model-Studio-key"
```

(`python3-opencv` and `python3-numpy` are commonly already present as ROS
dependencies; `python3-skimage` usually needs installing explicitly.)

## Pipeline

```
Config profile (configs/*.json)
  -> Qwen line-art transform OR blurred/contrast-enhanced local outline
  -> exact generated-ink threshold -> border artifact suppression
  -> skeleton -> structural and short facial-feature paths
  -> original-photo Canny details -> strength/proximity/isolation ranking
  -> remove detail segments already covered by structural paths
  -> arc-length Gaussian smoothing in canvas millimeters -> simplification
  -> exact-overlap joins + per-phase greedy travel ordering
  -> generate_sketch_paths.py        -> output/<config-name>/<painting_paths_file> (+ preview SVG)
```

Unlike `mondrian/`, there is no intermediate `painting_plan.json` step —
there's no vector design to describe first, so `generate_sketch_paths.py`
goes straight from traced pixel strokes to the final command list. It emits
one `select_tool` and one `dip_paint`, all `sketch_outline_*` paths, then all
`sketch_detail_*` paths. Travel is optimized independently within each phase.

### Usage

```bash
python3 Image_Process/sketch/generate_sketch_paths.py --config configs/sketch_demo_a4.json
python3 Image_Process/sketch/generate_sketch_paths.py --config configs/sketch_demo_4k_landscape.json
```

The `sketch_demo_4k_landscape.json` profile targets 4K landscape paper at
525 mm wide by 375 mm high. The A4 profile remains the CLI default.

`--config` defaults to `configs/sketch_demo_a4.json`. Run from the repo
root (paths in the config are CWD-relative), same as the mondrian scripts.
`--saved-semantic-image <path>` is the explicit offline retrace mode: it reads
that image, bypasses `generate_outline_image` and all network/API work, and
fails if the file is absent, empty, or undecodable. Successful reuse is
recorded as semantic status `reused` with retrace provenance from the config.

### Config fields

| Field | Meaning |
| --- | --- |
| `canvas.width_mm` / `height_mm` / `margin_mm` / `origin` | Same meaning as the mondrian config schema — `origin` must be `"top-left"`, `margin_mm` keeps the traced drawing inside the paper edge. |
| `source_image.path` | Path to the source bitmap (grayscale is read regardless of the file's color mode, with EXIF orientation normalized first). |
| `source_image.canny_threshold1` / `canny_threshold2` | Legacy local-outline Canny sensitivity used when the more specific outline settings are absent. |
| `source_image.min_stroke_length_mm` | After compatible strokes have been joined, drop residual shorter marks. Default 2.0mm; set 0 to disable. |
| `source_image.simplify_epsilon_ratio` | Douglas-Peucker simplification strength as a fraction of each stroke's arc length — larger = fewer points/straighter lines. |
| `source_image.simplify_min_epsilon_px` | Absolute simplification floor in source pixels. Default 1.0px to suppress raster stair-step jitter; set 0 for ratio-only behavior. |
| `semantic_preprocessing.enabled` | Use Qwen for the structural pass. Defaults to true; the web upload form exposes the same choice. Keeping it enabled permits the source image to be sent to Alibaba Cloud Model Studio. |
| `semantic_preprocessing.provider` / `model` / `prompt` | Provider is `"qwen"`; model and line-art conversion prompt are editable. The default model is `qwen-image-2.0-pro`; use another image-edit-capable Qwen model ID such as `qwen-image-edit-plus` when needed. |
| `semantic_preprocessing.timeout_seconds` / `fallback_to_local` | API deadline (default 60 seconds) and whether setup, safety, quota, timeout, malformed-response, or API failures continue through the local outline pass. |
| `outline_extraction.canny_threshold1` / `canny_threshold2` / `blur_kernel_size` / `contrast_clip_limit` | Local structural-outline controls. They are ignored for the generated line-art mask except during fallback. |
| `outline_extraction.line_art_threshold` / `min_path_length_mm` | Exact grayscale cutoff for generated ink (default 210) and structural-path floor (default 1mm). The direct cutoff preserves Qwen's gray anti-aliased thin lines; the short floor retains teeth, nostrils, nose contours, and lip marks. |
| `outline_extraction.border_suppression_band_px` | Removes connected components confined to the outer source-image band. Contours that enter the interior are preserved. |
| `detail_extraction.canny_threshold1` / `canny_threshold2` / `blur_kernel_size` | Original-photo detail edge controls. Defaults are 120/240 with a 5px Gaussian kernel to reject weak texture. |
| `detail_extraction.min_path_length_mm` / `max_paths` | Detail floor and budget. The floor cannot be configured below 0.5mm; the default budget is 400 paths. The budget is a maximum, so smaller candidate sets are not padded. |
| `detail_extraction.deduplication_distance_mm` | Optional explicit coverage distance for removing detail segments already represented by outlines. When omitted, the distance is 75% of the pen width capped at 0.75mm. Only covered segments are removed; gaps are never bridged. |
| `detail_extraction.connection_distance_mm` / `isolation_distance_mm` / `isolated_speckle_max_length_mm` | Ranking and isolated-speckle rejection controls. |
| `path_generation.tool_width_mm` | Pen stroke width (preview rendering + `path_settings.tool_width_mm` in the output). |
| `path_generation.home_position_mm` | Where the pen starts before the first stroke; affects greedy-ordering's first pick. Optional, defaults to the canvas's top-left margin corner. |
| `path_generation.join_overlapping_strokes` | If true (default), join compatible open strokes only at exactly equal mapped endpoints; never connect nearby endpoints through blank space. |
| `path_generation.max_join_turn_degrees` | Maximum heading change for a join (default 60°). Straight continuations win at multi-way junctions, while side branches remain separate. |
| `path_generation.smoothing_radius_mm` | Gaussian sigma in canvas millimeters, applied to dense centerlines before Douglas-Peucker simplification. The checked-in profile uses 0.5mm; 0 disables smoothing. Open endpoints/shared junction anchors stay fixed, closed paths are cyclic, and short paths cap the effective radius at one sixth of their length. |
| `path_generation.continuous_paths` | When true, emit one `paint_path` for each traced line. The checked-in profile enables it; false emits the legacy `paint_stroke` segment sequence. |
| `output.directory`, `.painting_paths_file`, `.preview_svg_file` | Same meaning as the mondrian config schema. |
| `output.semantic_outline_image_file` | Qwen result, or the local outline mask after fallback/disable, saved beside the preview for diagnosis. |

The checked-in values are balanced for likeness versus smooth execution. Lower
`min_stroke_length_mm`, `simplify_min_epsilon_px`, and
`max_join_turn_degrees` for a more conservative trace; raise them gradually for
fewer, bolder gestures. No setting allows joins across blank space.

New fields are optional so older sketch configs remain valid. Specifically,
missing smoothing/continuous fields resolve to the legacy 0mm and segmented
behavior; the checked-in profile and webapp write their newer defaults
explicitly.

## Semantic preprocessing and fallback

`semantic_preprocessing.py` calls this project's workspace-specific Beijing
DashScope HTTP endpoint (`ws-2xu46zkb1q2h7j9k.cn-beijing.maas.aliyuncs.com`)
and reads `DASHSCOPE_API_KEY`; credentials are never written
to config or output. The source image is sent as base64 and the returned image
is downloaded immediately because DashScope result URLs expire. Metadata records a
sanitized status (`used`, `reused`, `disabled`, or `local_fallback`) and reason code,
not API exception text. A missing key or package does not delay offline use.
When `fallback_to_local` is false, a categorized failure stops generation.

## canny_edges.py

The raster helpers normalize EXIF orientation, composite transparent pixels over white, run Canny or
generated-line-art thresholding, then `skimage.morphology.skeletonize`
to thin edges to single-pixel centerlines (avoiding the mirrored-duplicate
problem `cv2.findContours` has when it walks both sides of a stroke as one
blob boundary). Walks the skeleton pixel graph directly (endpoints/
junctions as nodes) to produce one entry per independent line — pixel
`(x, y)` points plus whether it's a closed loop (e.g. small blemish rings
in a source image). Adjacent junction pixels are clustered and their incident
endpoints share a central real skeleton pixel, eliminating micro-edges while
preserving exact source-ink overlap. Returns `(strokes, image_size)`.

`simplify(points_xy, closed, epsilon_ratio, min_epsilon_px)`:
Douglas-Peucker point reduction (`cv2.approxPolyDP`) on one stroke's pixel
points, with an absolute epsilon floor for short-path raster jitter.

**Note when swapping source images:** works best with a clean-background,
high-contrast image — busy backgrounds or low contrast flood the traced
strokes with tiny noise loops from Canny.

## path_ordering.py

Shared path continuity and greedy nearest-neighbor ordering.
`join_overlapping_strokes()` first joins the straightest compatible lines at
exact shared endpoints, reducing lower/lift cycles without creating connector
marks. `order_strokes(strokes_data,
home_position)` walks strokes starting from `home_position`, each time
picking whichever remaining stroke has an endpoint closest to the pen's
current position (reversing open strokes if that end is closer), to cut
down total pen-up travel. `total_travel_distance(strokes_points,
home_position)` sums the pen-up jumps for a given ordering — used to report
the before/after travel reduction.

## generate_sketch_paths.py

`build_two_phase_strokes()` creates and filters both phases, then scales pixel
points onto the canvas's drawable
box (canvas size minus `margin_mm` on each side), preserving aspect ratio
and centering the result — canvas and image both use a top-left, y-down
origin, so no axis flip is needed (unlike the `raymond` branch's
`"bottom-left"` option, which this route doesn't need since
`config_loader.py` requires `"top-left"`, same as `mondrian/`). Exact-overlap
joining happens before the structural minimum-length filter. Detail candidates
are scored by path length, source-edge strength, structural connection and
proximity, and isolation before the configured budget is applied.

`main()` orders each phase, then `build_commands()` turns each ordered stroke
into `move_to -> lower_tool -> paint_path -> lift_tool` in the checked-in
continuous mode. The robot can plan and retime the complete polyline without
stopping at every vertex. Setting `continuous_paths` false emits one straight
`paint_stroke` per consecutive point pair for hardware A/B comparison. A
single `select_tool`/`dip_paint` (black) is emitted once up front.

The output `debug` object includes source EXIF orientation and detail-working
dimensions, requested/effective smoothing radii, point counts before and
after smoothing/simplification, continuous/segmented command counts, the
effective deduplication distance, semantic status/fallback reason and retrace
provenance, plus per-phase raw/filtered/path/lift, paint-distance,
border-suppression, deduplication, and pen-up-travel metrics.

Before writing output, runs `path_validation.validate_painting_paths()` —
the route-local copy of the validator canonically maintained in
`image_to_mondrian/` — and stores the result
under `painting_paths["validation"]`; exits non-zero on validation errors,
same convention as the mondrian route.

## path_validation.py

Canonically maintained in `Image_Process/image_to_mondrian/path_validation.py`.
The route-local copy keeps this pipeline self-contained; behavioral changes
must be synchronized across the copies listed in `CLAUDE.md`.

## assets/apple.png

Reference source image, carried over from the `raymond` branch's
`assets/apple.png` — clean background, high contrast, good for exercising
the pipeline end to end.

## Tests

```bash
python3 -m unittest discover Image_Process/sketch/tests
```

The suite covers smoothing of open/closed/short/junction paths, continuous and
segmented commands with exact SVG geometry, border suppression, phase ordering,
detail ranking/budget, outline-detail deduplication, legacy configs, saved-image
reuse, and mocked Qwen success/fallback categories. A real Qwen call is
intentionally not part of the offline test suite.
