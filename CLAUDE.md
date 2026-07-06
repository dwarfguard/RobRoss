# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RobRoss (R.O.B Ross) is a robot-arm painting installation project: a robotic arm paints on a 12-inch canvas for public installations (malls, cafés). The project is in early prototype stage and spans three kinds of artifacts:

- **Mechanical design (CAD)** — SolidWorks assemblies for the physical hardware.
- **Image processing (Python)** — scripts that convert source images into paintable paths/instructions for the robot.
- **Planning docs** — design discussions and prototype decisions (bilingual English/Chinese).

There is no application build, lint, or test tooling yet — this is not a conventional software package.

## Repository Layout

- `README.md` — project abstract, target use case, and component breakdown (robot claw, canvas stand, brush, paint, paint bucket, control system, color mixer).
- `Rob_Ross_Discuss.md` — open design discussion: paint type, brush type, color mixing, art style tradeoffs (Blobs vs. Mondrian-inspired).
- `Rob_Ross_Prototype_v1.md` — the agreed first-prototype MVP spec (12" canvas, acrylic, premixed colors, Mondrian art style, preprocessed instructions rather than real-time AI decisions, ~30 min target completion time).
- `CAD/` — SolidWorks 2025 SP5.0 source files.
  - `CAD/prototypes/canvas_Holder/version_N/` — canvas frame/holder assemblies. Each versioned assembly (`Canvas_V{N}.SLDASM`) also depends on `CAD/prototypes/canvas_Holder/Canvas.SLDPRT`, which must be downloaded alongside the version folder to resolve the assembly.
  - `CAD/prototypes/paint_Holder/version_N/` — paint bucket/brush holder assemblies.
  - New design iterations should follow the existing `version_N/` subfolder convention rather than overwriting prior versions.
- `svg_path/canny.py` — early image-processing experiment using OpenCV's Canny edge detector (`cv2.Canny`) as a step toward converting source images into robot paint paths.
- `apple.png` — sample input image used by the image-processing scripts.
- `.venv/` — local Python 3.14 virtualenv (not committed logic, just the environment).

## Working with the Image-Processing Code

The Python side (`svg_path/`) is where image-to-path conversion logic lives, feeding the "Robot Control System" concept described in the planning docs (converting prepared artwork into robot movement paths). Dependencies are installed ad hoc (e.g. `pip install opencv-python`); there is no `requirements.txt` or `pyproject.toml` yet — check for one before assuming it's missing, and if adding real dependency management, prefer a `requirements.txt` at the repo root.

When extending image processing, keep in mind the two candidate art-style directions from `Rob_Ross_Discuss.md`:
- **Mondrian-inspired** (current prototype target): straight lines, rectangular regions, few colors — simplest to convert to robot paths.
- **Blobs**: region-based color grouping with white space between regions — a later-stage goal, more complex path generation.

## Working with CAD Files

CAD files are binary SolidWorks formats (`.SLDPRT`, `.SLDASM`) — they cannot be meaningfully read, diffed, or edited as text. Treat changes to these files as opaque binary updates from SolidWorks; do not attempt to parse or generate them programmatically unless explicitly asked to build tooling for that purpose.
