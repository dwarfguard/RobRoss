# Artwork Generation

Each subdirectory contains one self-contained artwork-generation approach with
its scripts, tests, and documentation.

| Module | Purpose | Status |
| --- | --- | --- |
| [mondrian](mondrian/README.md) | Generate Mondrian-style artwork, robot paths, and SVG previews. | Active Demo v1 pipeline |

New generation approaches should be added as sibling directories instead of
being folded into `mondrian/`.

Scripts are run from the repository root. They read profiles from `configs/`
and write generated files to `output/`; see the
[Mondrian pipeline guide](mondrian/README.md) for commands.
```text
Image_Process/
  mondrian/           Recursive-subdivision Mondrian-style line/block artwork
                      generator + path generator (see mondrian/README.md).
  sketch/             Canny-edge outline tracing of an arbitrary source image,
                      straight into painting_paths.json (see sketch/README.md).
  image_to_mondrian/  Photo quantized to a 5-color palette (the robot's actual
                      pens), segmented and fully filled, plus black grid lines
                      (see image_to_mondrian/README.md).
```

More algorithms are welcome as additional sibling folders here — each new
approach gets its own subfolder rather than growing inside an existing one.

`sketch/` and `image_to_mondrian/` are the subfolders with third-party
dependencies for image loading/quantization/edge-detection — `opencv-python`
+ `numpy` for both, plus `scikit-image` for `sketch/` only (its
skeletonization-based line tracer; `image_to_mondrian/` traces outlines via
`cv2.findContours` instead, so it doesn't need it). `mondrian/` stays pure
standard library. See `sketch/README.md` / `image_to_mondrian/README.md` for
the install command.

`mondrian/`, `sketch/`, and `image_to_mondrian/` all read their config
profiles from the repo-root `configs/` directory and write to the repo-root
`output/` directory (all CWD-relative paths in the scripts, so they're run
from the repo root — see each folder's README.md).
