# RobRoss Agent Guidance

RobRoss is an Aubo i5 robot-art prototype. The active Demo v1 target is a
supervised, pre-generated monochrome pen drawing on calibrated A4 paper.
Paint, automatic tool changes, audience interaction, and real-time generation
are outside the current hardware milestone.

## Read Only What You Need

When Serena tools are available, use them first for symbol discovery, reference
and call-relationship analysis, and targeted code retrieval. Read complete
source files only when their full context is required.

Use these documents as the authoritative sources; do not copy their commands,
calibration values, or parameter defaults into other documentation.

| Subject | Canonical source |
| --- | --- |
| Project orientation and workspace bootstrap | `README.md` |
| Requirements and scope | `docs/Rob_Ross_Prototype_v1.md` |
| Current hardware evidence and decisions | `docs/aubo-painting-current-status-2026-07-31.md` |
| Real-arm procedure | `docs/hardware_run_guide.md` |
| Path JSON contract | `docs/painting-paths-format.md` |
| Painter behavior | `ros2/robross_painter/README.md` |
| Painter parameters | `ros2/robross_painter/REFERENCE.md` |
| Artwork routes | `Image_Process/README.md` and each route's `README.md` |
| Eye-in-hand calibration and canvas localization | `eye_in_hand/README_EN.md` |

`eye_in_hand/README_EN.md` is canonical; the Chinese `eye_in_hand/README.md`
is a translation. Fixed-camera `handeye_calibration/` is retired and must not
be referenced as an active workflow.

## Architecture Constraints

- Artwork coordinates are millimeters from the canvas top-left. Only
  `ros2/robross_painter` translates them into robot motion.
- Keep artwork generation, path generation, validation, calibration, and robot
  execution separate.
- `select_tool` and `dip_paint` are abstract commands and currently no-op in
  the pen executor.
- A real-arm launch must use the calibrated robot model, an explicitly reviewed
  hardware profile, and a freshly qualified canvas. Never rely on RViz defaults.
- `MoveGroupInterface` spins its node internally on ROS 2 Humble; do not add the
  same node to another executor.
- Cartesian retiming starts from the trajectory's first waypoint because
  `getCurrentState()` may be stale while MoveIt's internal node spins.

## Copied Geometry Modules

`Image_Process/image_to_mondrian/` is canonical for `path_ordering.py` and
`path_validation.py`; copies live in `gemini_mondrian/`, `sketch/`, and
`line_art/`. It is also canonical for `border_tracing.py` and `region_fill.py`,
which are copied to `gemini_mondrian/`. Keep behavioral content synchronized.

Route modules use same-named top-level imports and can collide in
`sys.modules`; the webapp intentionally runs routes in subprocesses.

## Essential Commands

Run from the repository root:

```bash
python3 -m unittest discover Image_Process/mondrian/tests
python3 -m unittest discover Image_Process/image_to_mondrian/tests
python3 -m unittest discover Image_Process/gemini_mondrian/tests
python3 -m unittest discover Image_Process/line_art/tests
python3 -m unittest discover Image_Process/sketch/tests
python3 docs/check_docs.py
```

For ROS tests and hardware commands, use the canonical painter and hardware
guides rather than duplicating commands here.
