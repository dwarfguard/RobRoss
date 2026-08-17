# Artwork Generation

Each subdirectory contains one self-contained artwork-generation approach with
its scripts, tests, and documentation.

| Module | Purpose | Status |
| --- | --- | --- |
| [mondrian](mondrian/README.md) | Generate Mondrian-style artwork, robot paths, and SVG previews. | Active Demo v1 pipeline |
| [sketch](sketch/README.md) | Two-phase structural outline + ranked likeness details, with optional Qwen overlap enhancement. | Active |
| [image_to_mondrian](image_to_mondrian/README.md) | Photo quantized to 5-color palette, region-filled with black grid lines. | Active |
| [gemini_mondrian](gemini_mondrian/README.md) | Gemini image-to-image Mondrian restyle + standalone vectorization. | Active |
| [line_art](line_art/README.md) | Clean line-art/illustration tracing via threshold + skeletonize (not Canny). | Active |

New generation approaches should be added as sibling directories instead of
being folded into `mondrian/`.

Scripts are run from the repository root. They read profiles from `configs/`
and write generated files to `output/`; see the
[Mondrian pipeline guide](mondrian/README.md) for commands.
```text
Image_Process/
  mondrian/           Recursive-subdivision Mondrian-style line/block artwork
                      generator + path generator (see mondrian/README.md).
  sketch/             Two-phase photo sketch: optional Qwen structural line
                      art plus original-photo detail paths (see sketch/README.md).
  image_to_mondrian/  Photo quantized to a configurable palette, segmented and
                      scanline-filled within retained masks, plus black grid lines
                      (see image_to_mondrian/README.md).
  gemini_mondrian/    Gemini image-to-image Mondrian-style restyle, then its own
                      simplified quantize/segment/fill/border-trace pipeline
                      (see gemini_mondrian/README.md).
  line_art/           Already-clean line-art/technical-illustration tracing via
                      threshold + skeletonize (not Canny - avoids double-edge
                      artifacts on strokes with real width; see line_art/README.md).
```

More algorithms are welcome as additional sibling folders here — each new
approach gets its own subfolder rather than growing inside an existing one.

`sketch/`, `image_to_mondrian/`, `line_art/`, and `gemini_mondrian/` have
third-party dependencies for image processing or generation —
`opencv-python` + `numpy` for all three, plus `scikit-image` for `sketch/`
and `line_art/` (both use a skeletonization-based line tracer;
`image_to_mondrian/` traces outlines via `cv2.findContours` instead, so it
doesn't need it). `sketch/` also uses Pillow to normalize EXIF orientation.
`gemini_mondrian/` requires `google-genai`; `sketch/`
optionally uses the DashScope HTTP API and falls back locally when it is unavailable.
`mondrian/` stays pure
standard library. See each folder's README for
route-specific installation instructions.

Every route reads its config profiles from the repo-root `configs/`
directory and writes to the repo-root `output/` directory (all
CWD-relative paths in the scripts, so they're run from the repo root — see
each folder's README.md).
