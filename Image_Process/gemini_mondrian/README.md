# gemini_mondrian

A fourth artwork-generation route, alongside `Image_Process/mondrian/`,
`Image_Process/sketch/`, and `Image_Process/image_to_mondrian/`. This one
uses Gemini's image-to-image generation to redraw an arbitrary source photo
as a Mondrian-style **portrait** - the subject stays recognizably a person
(same pose/proportions), just stylized with flat white/black/red/blue/
yellow color blocks and bold black grid lines - then vectorizes that
generated image into the same `painting_paths.json` command format every
other route produces.

```text
Config profile (configs/gemini_mondrian_*.json)
  -> gemini_client.generate_styled_image()   photo -> Gemini-generated Mondrian-style image
  -> color_quantize.quantize_to_palette()    Gemini image -> 5-color label image
  -> segmentation.segment_image()            per-color connected regions, speckle filtered
  -> region_fill.region_to_pixel_strokes()   erode + scanline fill, per region
  -> border_tracing.trace_region_contours()  black grid lines between color blocks
  -> path_ordering.order_strokes()           greedy nearest-neighbor travel order
  -> output/<run>/painting_paths.json (+ preview SVG + quantized preview PNG)
```

## Why this route has its own vectorization code, not a shared one

An earlier version of this route subprocess-called
`Image_Process/image_to_mondrian/`'s pipeline unmodified for the
vectorization step. Real testing showed that badly distorts Gemini's
output: `image_to_mondrian`'s quantization/segmentation is tuned for **real
photographs** - bilateral filtering to suppress sensor noise, a large
morphological closing kernel to bridge gaps a photo's own shadows cut into
one object, an adaptive chroma threshold tuned to a photo's continuous color
distribution, MediaPipe face-landmark protection so that closing doesn't
eat facial detail. None of those problems exist in a Gemini-generated
image - it's already flat, bold-colored, and sharp-edged. Running real-photo
-strength smoothing/closing over an already-clean image just rounds
everything into an unrecognizable blob (confirmed directly: a clearly
recognizable Gemini portrait, quantized through `image_to_mondrian`'s
pipeline, became featureless color blobs with no face or hat visible).

So `color_quantize.py` and `segmentation.py` here are **deliberately
simpler** rewrites, not copies:

- No bilateral filtering / no neutral-chroma gating in `color_quantize.py` -
  plain nearest-palette-color matching, since Gemini's output is already
  close to pure palette colors.
- No morphological *closing* in `segmentation.py`, only a small *opening*
  pass to strip anti-aliasing/compression speckle. No face protection either
  - that machinery exists solely to counteract closing's side effects, and
  with no closing there's nothing to protect against.

`region_fill.py`, `border_tracing.py`, `path_ordering.py`, and
`path_validation.py` **are** copied byte-for-byte from `image_to_mondrian/`
- those are pure computational geometry (scanline fill, contour tracing,
greedy nearest-neighbor ordering, bounds checking) with no real-photo
assumptions baked in, so there was nothing to change.

Each route's helper modules (`config_loader`, `path_validation`, ...) are
same-named top-level imports duplicated per folder rather than shared via
cross-folder `import` - see the root `CLAUDE.md`'s "Architecture within"
sections for why (`sys.modules` collisions if two routes' code ever loaded
into the same process).

## Config schema

Unlike an earlier version, this is a single flat config - no
`downstream_template_config` indirection, since vectorization is no longer
delegated to another route's subprocess:

```json
{
  "canvas": {"width_mm": 210.0, "height_mm": 297.0, "margin_mm": 10.0, "origin": "top-left"},
  "source_image": {"path": "...", "downscale_max_dimension_px": 1024},
  "gemini": {"model": "gemini-2.5-flash-image", "prompt": "..."},
  "palette": {"colors": [...], "color_space": "lab"},
  "segmentation": {"min_region_area_mm2": 30.0, "morph_open_kernel_px": 3, "skip_white_regions": true},
  "path_generation": {"tool_width_mm": 3.0, "stroke_overlap_ratio": 0.15, "mask_erosion_mm": 1.0, "home_position_mm": [10.0, 10.0]},
  "border_generation": {"draw_borders": true, "simplify_epsilon_ratio": 0.0015},
  "output": {"directory": "...", "gemini_output_image_file": "gemini_styled.png", "painting_paths_file": "...", "preview_svg_file": "...", "quantized_preview_png_file": "..."}
}
```

`min_region_area_mm2`/`morph_open_kernel_px`/`simplify_epsilon_ratio` are
starting points verified against real Gemini output (see "Verified
against real output" below) but are not universal constants - re-check
visually (`quantized_preview_png_file` and `preview_svg_file`) against any
new source photo, the same as every other route's tuning workflow.

## Setup

```bash
pip install google-genai
export GEMINI_API_KEY=your-key-here
```

`google-genai` is this route's only new dependency, lazily imported inside
`gemini_client.generate_styled_image()` - the rest of this module (and the
rest of the repo) doesn't need it installed. The API key is read from the
`GEMINI_API_KEY` environment variable only, never written into a config file
or committed anywhere. No `mediapipe` dependency here (unlike
`image_to_mondrian`) - this route doesn't need face protection.

## Usage

```bash
python3 Image_Process/gemini_mondrian/generate_painting_paths.py \
  --config configs/gemini_mondrian_demo_a4.json
```

`--config` defaults to `configs/gemini_mondrian_demo_a4.json`. Run from the
repo root, same as every other route.

Outputs (under `output/<config-name>/`):

- `gemini_styled.png` - the Gemini-generated Mondrian-style image.
- `<painting_paths_file>` - the final command JSON, `style: "gemini_mondrian"`,
  with `gemini_image_file`/`gemini_model` fields recording what was
  actually generated.
- `<preview_svg_file>` - rendered paint strokes + dashed travel lines.
- `<quantized_preview_png_file>` - every pixel painted its quantized
  palette color, the fastest way to check region/detail recognizability
  before running the full fill/border-trace pipeline.

## Verified against real output

Confirmed end to end against a real Gemini API call (not just unit tests):
the quantized preview and final path preview both keep the subject's face,
eyes, lips, and hat clearly recognizable, closely matching the Gemini
source image. This directly disproved the earlier
subprocess-to-`image_to_mondrian` approach, which turned the same source
image into unrecognizable blobs.

## Not (yet) further tuned

`gemini_client.py`'s Gemini SDK call (`contents=[prompt, image part]`,
reading generated bytes back out of
`response.candidates[0].content.parts[*].inline_data`) is confirmed working
against the real API. Border-stroke count on the verified Lenna run was
higher (~300) than `image_to_mondrian`'s heavily-closed real-photo runs
(~20-100) - each kept region's boundary detail (eyebrows, individual hair
strands) gets its own border stroke since there's no closing pass merging
them away. That's the direct trade-off for keeping the extra recognizable
detail; if stroke count needs to come down for a specific painting session,
raising `min_region_area_mm2` or `morph_open_kernel_px` slightly is the
first thing to try (re-verify visually after any change).
