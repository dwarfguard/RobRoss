# painting_paths.json Format

Format reference for `output/<config-name>/painting_paths.json`, produced by
`Image_Process/mondrian/generate_painting_paths.py` and validated by `Image_Process/mondrian/path_validation.py`.

## Coordinate system

- Units: millimeters (`units: "mm"`).
- Origin `(0, 0)`: top-left (`canvas.origin == "top-left"`).
- `x` increases right, `y` increases down.
- All coordinates should fall within `[0, canvas.width_mm]` x `[0, canvas.height_mm]`.

## Command types

Each command is a dict with a `"command"` field, an optional `"label"`
(never required, for debugging only), and type-specific fields.

| Command | Required fields | Notes |
|---------|------------------|-------|
| `select_tool` | `color` | Selects paint color/tool. |
| `dip_paint` | `color` | Dips tool in paint. |
| `move_to` | `x_mm`, `y_mm` (numeric) | Moves tool (lifted) to a point. |
| `lower_tool` | — | No coordinates needed. |
| `paint_stroke` | `from_mm`, `to_mm` (two-number lists), `color` | Straight stroke while tool is down. |
| `paint_path` | `points_mm` (list of >= 2 two-number lists), `color` | Continuous polyline while tool is down: the tool draws through every point without lifting or stopping. |
| `lift_tool` | — | No coordinates needed. |

Any other `"command"` value is unknown: forward-compatible so new command
types don't break existing pipelines. The validator flags it as a warning
(not an error), and the ROS 2 executor skips it with a warning at both
preflight and execution time (neither rejects the file).

Typical sequence per continuous line: `move_to` -> `lower_tool` ->
`paint_path` -> `lift_tool`, with `select_tool`/`dip_paint` emitted once per
color group. `paint_stroke` remains valid for single straight strokes
(e.g. the test line).

## Assumptions

- Commands execute strictly in list order.
- A `paint_stroke` is one straight line. A `paint_path` is a continuous
  polyline; curved lines are represented by sampling the curve densely
  into `points_mm` (no separate curve command needed).
- Command order is chosen by the path optimizer (continuous polylines,
  chained touching lines, nearest-neighbor travel), not by the artwork
  creation order. Only fills-before-lines is guaranteed.
- `label` is for humans/logging only — nothing should key behavior off it.
- Coordinates are pre-rounded to 2 decimals by the generator.

## Validation rules

`path_validation.validate_painting_paths(painting_paths)` returns
`{"passed": bool, "errors": [...], "warnings": [...]}`. `passed` is true iff
`errors` is empty; warnings never affect it.

**Canvas:**
- Must exist; `width_mm`/`height_mm` must be present and positive — error otherwise.
- `origin` should be `"top-left"` — warning if missing or different.

**Every command:**
- Must have a `"command"` field — error otherwise.
- Unknown command type — warning.
- `move_to` must have numeric `x_mm`/`y_mm`, inside canvas bounds — error
  otherwise (the robot physically travels to `move_to` targets, so
  out-of-bounds travel is as dangerous as an out-of-bounds stroke).
- `select_tool`/`dip_paint`/`paint_stroke`/`paint_path` should have `color` — warning if missing.
- `paint_stroke` must have `from_mm`/`to_mm`, each a two-number list, both inside
  canvas bounds, with nonzero distance between them — error otherwise.
- `paint_path` must have `points_mm` with at least 2 two-number points, all
  inside canvas bounds, with nonzero total length — error otherwise. A
  zero-length segment (duplicate consecutive point) is a warning.
- `lower_tool`/`lift_tool` need no coordinates.

**Command sequence (pen-state machine):**

`validate_command_sequence` simulates the pen's up/down state across the whole
command list, because commands that are each individually valid can still form
an illegal sequence. The robot executes commands strictly in order and physically
refuses these transitions at runtime — but only *after* it has already executed
every command before the bad one, so without this check the arm moves and lowers
the pen before discovering the sequence is malformed. The ROS 2 executor
(`painting_executor`) runs the same state machine in its file preflight, so a
malformed file is rejected before any motion. Unknown commands are treated as
skipped no-ops and do not affect pen state.

- `move_to` while the pen is down — error (travel drags the pen; `lift_tool` first).
- `lower_tool`/`lift_tool` before any `move_to` has positioned the pen — error.
- `paint_stroke`/`paint_path` while the pen is up — error (`lower_tool` first).
- The command sequence must end with the pen up — a final `lift_tool` returns the
  pen to safe height; ending with the pen down is an error.
- A redundant `lower_tool` (pen already down) or `lift_tool` (pen already up) is a
  warning.
