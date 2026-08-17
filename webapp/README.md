# webapp

A small local control panel on top of the repo's existing generation
pipelines: pick a route, optionally upload a photo, click Process, and browse
the result in the same card layout `generate_output_gallery.py` already
produces for `output/index.html`.

This is a thin orchestration layer, not a rewrite - it drives the existing
per-route CLI scripts (`Image_Process/*/generate_*.py`) exactly the way the
rest of the docs describe running them, just from a web form instead of a
terminal. It doesn't change any route's own generation logic.

## Install & run

```bash
pip install flask
python3 webapp/app.py
```

Then open **http://127.0.0.1:5050**.

This is a **local dev tool only**: no authentication, binds to `127.0.0.1`
(localhost) on purpose. Do not change the host to `0.0.0.0` or otherwise
expose it to a network - anyone who can reach it can run arbitrary generation
jobs and read anything under `output/`.

## What happens when you click Process

1. A new `output/upload_<timestamp>_<route>/` directory is created.
2. If the route needs a source image (every route except `mondrian`), the
   upload is saved into that same directory as `upload_<original-filename>`
   - uploads never get mixed into `Image_Process/assets/`, which is reserved
   for the repo's own curated sample images (documented in the root
   `CLAUDE.md` config profile table).
3. A one-off config is cloned from that route's existing template config
   (e.g. `configs/image_to_mondrian_demo_a4.json`) with `output.directory`
   (and `source_image.path`, if applicable) pointed at the new run folder,
   and written to `output/upload_.../\_config.json` - **not** into `configs/`,
   which stays reserved for the hand-maintained profile list.
   For the sketch route, the visible **Enhance overlapping forms with Qwen**
   checkbox is enabled by default and the **Qwen image model** defaults to
   `qwen-image-2.0-pro`. The form also exposes the 0–255 generated-line-art
   threshold (210), semantic outline floor (1.0mm), smoothing radius (0.5mm),
   continuous paths (enabled), and maximum detail paths (400). Every choice is
   recorded in this config. The path budget must be a
   non-negative integer and remains only a maximum, not a target stroke count.
   Leave it checked only when the uploaded image may be sent to Alibaba Cloud.
   Qwen failures still use the sketch route's configured local fallback.
   Sketch and line-art routes also expose a paper-size selector. A4 remains
   the default; **4K landscape** selects the 525 mm by 375 mm profile.
4. The route's existing script(s) run as subprocesses against that config,
   exactly like `python3 Image_Process/<route>/generate_*.py --config <path>`
   from the README/CLAUDE.md docs. If a step fails, its stderr is shown on
   an error page - nothing is swallowed.
5. `generate_output_gallery.py`'s `collect_runs()` picks the new run up
   automatically (it looks for a `configs/<name>.json` OR the run's own
   `_config.json`), so it shows up as a new card without any extra
   bookkeeping.

## Retracing a saved Qwen image

Eligible sketch cards have a **Retrace saved Qwen image** panel prefilled from
that run's config. Submitting it creates a distinct
`output/retrace_<timestamp>_sketch/` run, copies the original photo and saved
Qwen bytes, and regenerates JSON, SVG, validation, and gallery state with the
new threshold, outline floor, smoothing, continuous-mode, and detail-budget
settings. The source run is never changed.

Retracing invokes the sketch CLI's explicit `--saved-semantic-image` mode, so
it never calls Qwen or another network API. The server validates the run name,
resolved `output/` containment, sketch style, saved config, source photo,
painting-path semantic status, and semantic image before copying anything.
Only runs whose status is `used` or `reused` are eligible; local-fallback and
disabled runs do not claim their intermediate is a saved Qwen result. A
missing or unreadable intermediate fails closed instead of falling back to a
new Qwen request.

## Deleting a run

Every card in the gallery has a **Delete run** button. It deletes the whole
`output/<name>/` folder in one go - not individual files inside it, since a
run's `painting_paths.json` and its preview files are one coherent unit
(deleting just one would leave the rest orphaned and broken in the gallery,
per `generate_output_gallery.py`'s `collect_runs()`).

Clicking it pops a browser `confirm()` dialog first - there's no undo, no
trash/recycle bin, and no persistent database to restore from (see "Not in
scope" below), so a mis-click is unrecoverable. After deleting,
`output/index.html` is regenerated automatically so the static gallery
(opened directly via `file://`, without the webapp running) doesn't keep
showing a card for a run that no longer exists.

The delete button only appears when the webapp renders the gallery - the
static `output/index.html` produced by `python3 generate_output_gallery.py`
never shows one, since that page has no server behind it to handle the
delete request.

## Adding a new route

Everything route-specific lives in `route_adapters.py`'s `ROUTE_ADAPTERS`
dict - one route generation approach (see the root `CLAUDE.md`'s "one
subfolder per artwork-generation algorithm" convention) maps to one entry:
a display label, which existing config to clone as a template, whether it
needs an uploaded photo, and the ordered list of scripts to run. Adding a
future route needs one new dict entry here - nothing in `app.py` or
`generate_output_gallery.py` needs to change.

## Not in scope here

Except for the sketch controls documented above, there is no broader parameter
tuning UI (palette, morphology kernel sizes, Canny thresholds,
...) - every run uses its route's template config's defaults unchanged. No
persistent database - `output/*/` itself, browsable through the gallery, is
the "database". No auth, no multi-user isolation, no background job queue -
processing blocks the request until the subprocess(es) finish, which is fine
for local single-person use.
