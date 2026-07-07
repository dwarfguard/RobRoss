# painting_paths.json Format

Format reference for `output/painting_paths.json`, produced by
`scripts/generate_painting_paths.py` and validated by `scripts/path_validation.py`.

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
| `lift_tool` | — | No coordinates needed. |

Any other `"command"` value is unknown: flagged as a validation warning, not
an error, so new command types don't break existing pipelines.

Typical sequence per stroke: `move_to` -> `lower_tool` -> `paint_stroke` -> `lift_tool`,
with `select_tool`/`dip_paint` emitted once per color before its first stroke group.

## Assumptions

- Commands execute strictly in list order.
- A `paint_stroke` is one straight line; curves are multiple strokes.
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
- `select_tool`/`dip_paint`/`paint_stroke` should have `color` — warning if missing.
- `paint_stroke` must have `from_mm`/`to_mm`, each a two-number list, both inside
  canvas bounds, with nonzero distance between them — error otherwise.
- `lower_tool`/`lift_tool` need no coordinates.
