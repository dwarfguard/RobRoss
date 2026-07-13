# Image_Process

Image processing / artwork-generation module. Each subfolder is one
generation algorithm ("path"), self-contained with its own scripts, tests,
and README:

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
dependencies (`opencv-python`, `numpy`, `scikit-image`, for image
loading/quantization/edge-detection/skeletonization) — `mondrian/` stays
pure standard library. See `sketch/README.md` / `image_to_mondrian/README.md`
for the install command.

`mondrian/`, `sketch/`, and `image_to_mondrian/` all read their config
profiles from the repo-root `configs/` directory and write to the repo-root
`output/` directory (all CWD-relative paths in the scripts, so they're run
from the repo root — see each folder's README.md).
