#!/usr/bin/env python3
"""Scan output/<config-name>/ and write output/index.html, a static gallery
for eyeballing every generated route/config's result at once instead of
manually opening files one by one. Run after (re-)generating any artwork:

    python3 generate_output_gallery.py

Then open output/index.html directly in a browser (no server needed - it
only uses relative <img>/<a> paths, which load fine under file://). Pure
standard library, no dependency on any route's own code.
"""

import json
from html import escape
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
CONFIGS_DIR = REPO_ROOT / "configs"
OUTPUT_DIR = REPO_ROOT / "output"
INDEX_FILE = OUTPUT_DIR / "index.html"


def resolve_output_files(output_config: dict) -> dict:
    """Map an ambiguous, route-specific `output.*` config schema onto a
    fixed set of roles. `path_preview_svg_file` only exists for the
    mondrian route (which also has a separate `preview_svg_file` artwork
    preview); sketch/image_to_mondrian reuse `preview_svg_file` itself as
    the path preview since they have no separate artwork-generation stage.
    """
    files = {}
    if "painting_paths_file" in output_config:
        files["paths_json"] = output_config["painting_paths_file"]
    if "path_preview_svg_file" in output_config:
        files["path_preview_svg"] = output_config["path_preview_svg_file"]
        if "preview_svg_file" in output_config:
            files["artwork_preview_svg"] = output_config["preview_svg_file"]
    elif "preview_svg_file" in output_config:
        files["path_preview_svg"] = output_config["preview_svg_file"]
    if "path_animation_svg_file" in output_config:
        files["animation_svg"] = output_config["path_animation_svg_file"]
    if "quantized_preview_png_file" in output_config:
        files["quantized_png"] = output_config["quantized_preview_png_file"]
    return files


def _load_run(name: str, run_dir: Path, config: dict) -> dict:
    files = resolve_output_files(config["output"])
    if "paths_json" not in files:
        return None
    paths_json_path = run_dir / files["paths_json"]
    if not paths_json_path.is_file():
        return None
    painting_paths = json.loads(paths_json_path.read_text(encoding="utf-8"))
    return {
        "name": name,
        "run_dir": run_dir,
        "config": config,
        "files": files,
        "painting_paths": painting_paths,
        # Proxy for "when was this generated" - the main artifact's mtime,
        # not a separately-tracked field, so nothing new needs maintaining.
        "created_ts": paths_json_path.stat().st_mtime,
    }


def collect_runs() -> list:
    """Two sources of runs, merged: hand-curated profiles in `configs/*.json`
    (matched to `output/<config-stem>/`), and ad-hoc runs (e.g. from
    webapp/'s uploads) that carry their own `output/<name>/_config.json`
    instead of a `configs/` entry - `configs/` stays reserved for the
    documented, hand-maintained profile list."""
    runs = []
    seen_names = set()

    for config_path in sorted(CONFIGS_DIR.glob("*.json")):
        name = config_path.stem
        run_dir = OUTPUT_DIR / name
        if not run_dir.is_dir():
            continue
        config = json.loads(config_path.read_text(encoding="utf-8"))
        run = _load_run(name, run_dir, config)
        if run:
            runs.append(run)
            seen_names.add(name)

    for run_dir in sorted(OUTPUT_DIR.iterdir()):
        if not run_dir.is_dir() or run_dir.name in seen_names:
            continue
        generated_config_path = run_dir / "_config.json"
        if not generated_config_path.is_file():
            continue
        config = json.loads(generated_config_path.read_text(encoding="utf-8"))
        run = _load_run(run_dir.name, run_dir, config)
        if run:
            runs.append(run)

    return runs


def render_stats_table(debug: dict) -> str:
    rows = "".join(
        f"<tr><td>{escape(str(key))}</td><td>{escape(str(value))}</td></tr>"
        for key, value in debug.items()
    )
    return f"<table class='stats'>{rows}</table>"


def render_validation_badge(validation: dict) -> str:
    if validation.get("passed"):
        return "<span class='badge badge-pass'>validation passed</span>"
    errors = "".join(f"<li>{escape(str(e))}</li>" for e in validation.get("errors", []))
    return (
        "<span class='badge badge-fail'>validation FAILED</span>"
        f"<ul class='errors'>{errors}</ul>"
    )


def render_card(run: dict, base_url: str = "", show_delete: bool = False) -> str:
    """base_url prefixes every file link/src - "" (default) gives paths
    relative to output/index.html itself, which is how the static CLI usage
    works under file://. webapp/app.py passes "/output/" instead, since it
    serves the gallery page at "/" but the actual files under "/output/".

    show_delete adds a delete button posting to "{base_url}{name}/delete".
    Defaults to False so the static output/index.html (generated by main(),
    opened via file://, no backend to handle the POST) never gets a button
    that does nothing when clicked - only webapp/app.py, which actually
    implements that route, turns it on."""
    name = run["name"]
    run_dir = run["run_dir"]
    files = run["files"]
    painting_paths = run["painting_paths"]
    canvas = painting_paths.get("canvas", {})
    style = painting_paths.get("style", "?")
    validation = painting_paths.get("validation", {"passed": None, "errors": []})
    debug = painting_paths.get("debug", {})

    rel = lambda filename: f"{base_url}{name}/{filename}"

    preview_html = ""
    if "quantized_png" in files:
        preview_html += (
            f"<a href='{escape(rel(files['quantized_png']))}' target='_blank'>"
            f"<img class='thumb' src='{escape(rel(files['quantized_png']))}' "
            f"alt='{escape(name)} quantized preview'></a>"
        )
    if "path_preview_svg" in files:
        preview_html += (
            f"<a href='{escape(rel(files['path_preview_svg']))}' target='_blank'>"
            f"<img class='thumb' src='{escape(rel(files['path_preview_svg']))}' "
            f"alt='{escape(name)} path preview'></a>"
        )
    if "artwork_preview_svg" in files:
        preview_html += (
            f"<a href='{escape(rel(files['artwork_preview_svg']))}' target='_blank'>"
            f"<img class='thumb' src='{escape(rel(files['artwork_preview_svg']))}' "
            f"alt='{escape(name)} artwork preview'></a>"
        )

    links = [f"<a href='{escape(rel(files['paths_json']))}'>painting_paths.json</a>"]
    if "animation_svg" in files:
        links.append(f"<a href='{escape(rel(files['animation_svg']))}'>animation</a>")

    delete_html = ""
    if show_delete:
        delete_html = f"""
  <form class="card-actions" method="post" action="{escape(base_url)}{escape(name)}/delete"
        onsubmit="return confirm('Delete output/{escape(name)}/? This cannot be undone.');">
    <button type="submit" class="delete-btn">Delete run</button>
  </form>"""

    return f"""
<section class="card" data-name="{escape(name.lower())}" data-style="{escape(style)}"
          data-passed="{'true' if validation.get('passed') else 'false'}"
          data-created="{run['created_ts']}">
  <h2>{escape(name)} <span class="style-badge">{escape(style)}</span></h2>
  <p class="canvas">{escape(str(canvas.get('width_mm', '?')))}mm x
     {escape(str(canvas.get('height_mm', '?')))}mm</p>
  {render_validation_badge(validation)}
  <div class="previews">{preview_html}</div>
  {render_stats_table(debug)}
  <p class="links">{' | '.join(links)}</p>
  <p class="run-dir">output/{escape(name)}/</p>{delete_html}
</section>
"""


STYLE = """
body { font-family: -apple-system, sans-serif; background: #f4f4f4; margin: 0; padding: 2rem; }
h1 { margin-top: 0; }
.toolbar { background: white; border-radius: 8px; padding: 0.75rem 1rem; margin-bottom: 1.5rem; box-shadow: 0 1px 3px rgba(0,0,0,0.15); display: flex; gap: 0.75rem; flex-wrap: wrap; }
.toolbar input, .toolbar select { padding: 0.4rem 0.6rem; border: 1px solid #ddd; border-radius: 4px; font-size: 0.85rem; }
.toolbar input { flex: 1; min-width: 160px; }
.grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(340px, 1fr)); gap: 1.5rem; }
.card { background: white; border-radius: 8px; padding: 1rem 1.25rem; box-shadow: 0 1px 3px rgba(0,0,0,0.15); }
.card h2 { margin: 0 0 0.25rem 0; font-size: 1.1rem; }
.style-badge { font-size: 0.7rem; font-weight: normal; background: #eee; border-radius: 4px; padding: 0.1rem 0.4rem; color: #555; }
.canvas { margin: 0 0 0.5rem 0; color: #666; font-size: 0.85rem; }
.badge { display: inline-block; font-size: 0.75rem; border-radius: 4px; padding: 0.15rem 0.5rem; margin-bottom: 0.5rem; }
.badge-pass { background: #d6f5d6; color: #1a7a1a; }
.badge-fail { background: #f8d6d6; color: #a11a1a; }
.errors { color: #a11a1a; font-size: 0.8rem; }
.previews { display: flex; gap: 0.5rem; flex-wrap: wrap; margin-bottom: 0.5rem; }
.thumb { max-width: 150px; max-height: 200px; border: 1px solid #ddd; border-radius: 4px; }
table.stats { width: 100%; border-collapse: collapse; font-size: 0.8rem; margin-bottom: 0.5rem; }
table.stats td { padding: 0.15rem 0.4rem; border-bottom: 1px solid #eee; }
table.stats td:first-child { color: #666; }
table.stats td:last-child { text-align: right; font-variant-numeric: tabular-nums; }
.links { font-size: 0.8rem; }
.run-dir { font-size: 0.75rem; color: #999; font-family: monospace; }
.card-actions { margin: 0.5rem 0 0 0; }
.delete-btn { padding: 0.3rem 0.7rem; border: 1px solid #f3c2c2; border-radius: 4px; background: #fff0f0; color: #a11a1a; font-size: 0.8rem; cursor: pointer; }
.delete-btn:hover { background: #fde0e0; }
"""


def render_toolbar(runs: list) -> str:
    """Search/filter/sort controls, pure client-side (no fetch/XHR, which
    file:// blocks) - just shows/hides and reorders the already-rendered
    .card elements via their data-* attributes. Route options are built from
    the styles actually present in `runs`, not a hardcoded enum, so a future
    fourth route shows up automatically."""
    styles = sorted({run["painting_paths"].get("style", "?") for run in runs})
    style_options = "".join(f"<option value='{escape(s)}'>{escape(s)}</option>" for s in styles)
    return f"""
<div class="toolbar">
  <input type="text" id="gallery-search" placeholder="Search by name...">
  <select id="gallery-style-filter">
    <option value="">All routes</option>
    {style_options}
  </select>
  <select id="gallery-validation-filter">
    <option value="">All validation</option>
    <option value="true">Passed</option>
    <option value="false">Failed</option>
  </select>
  <select id="gallery-sort">
    <option value="created-desc">Newest first</option>
    <option value="created-asc">Oldest first</option>
    <option value="name-asc">Name A-Z</option>
  </select>
</div>
<script>
document.addEventListener('DOMContentLoaded', function() {{
  // Deferred to DOMContentLoaded because this <script> is emitted (and
  // would otherwise run) before <div class="grid"> exists in the page -
  // querying it eagerly here returns null and silently no-ops every filter.
  var search = document.getElementById('gallery-search');
  var styleFilter = document.getElementById('gallery-style-filter');
  var validationFilter = document.getElementById('gallery-validation-filter');
  var sortSelect = document.getElementById('gallery-sort');
  var grid = document.querySelector('.grid');
  if (!grid) return;

  function applyFilters() {{
    var query = search.value.trim().toLowerCase();
    var styleVal = styleFilter.value;
    var validationVal = validationFilter.value;
    var cards = Array.prototype.slice.call(grid.querySelectorAll('.card'));

    cards.forEach(function(card) {{
      var matchesQuery = !query || card.dataset.name.indexOf(query) !== -1;
      var matchesStyle = !styleVal || card.dataset.style === styleVal;
      var matchesValidation = !validationVal || card.dataset.passed === validationVal;
      card.style.display = (matchesQuery && matchesStyle && matchesValidation) ? '' : 'none';
    }});

    var sortVal = sortSelect.value;
    cards.sort(function(a, b) {{
      if (sortVal === 'created-desc') return b.dataset.created - a.dataset.created;
      if (sortVal === 'created-asc') return a.dataset.created - b.dataset.created;
      if (sortVal === 'name-asc') return a.dataset.name.localeCompare(b.dataset.name);
      return 0;
    }});
    cards.forEach(function(card) {{ grid.appendChild(card); }});
  }}

  [search, styleFilter, validationFilter, sortSelect].forEach(function(el) {{
    el.addEventListener('input', applyFilters);
    el.addEventListener('change', applyFilters);
  }});
  applyFilters();
}});
</script>
"""


def render_gallery_grid(runs: list, base_url: str = "", show_delete: bool = False) -> str:
    """The toolbar + cards-in-a-grid markup, no <html>/<head>/<body> wrapper -
    reused as-is by webapp/app.py so both the static CLI and the Flask app
    render identical cards (and the same search/filter/sort behavior) from
    the same code. See render_card() for what base_url and show_delete do."""
    cards = "\n".join(render_card(run, base_url, show_delete) for run in runs)
    return f'{render_toolbar(runs)}\n<div class="grid">\n{cards}\n</div>'


def build_html(runs: list, extra_body_html: str = "") -> str:
    """extra_body_html is injected right after the intro paragraph - used by
    webapp/app.py to add its upload form without duplicating the page shell
    or STYLE block."""
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>RobRoss output gallery</title>
<style>{STYLE}</style>
</head>
<body>
<h1>RobRoss output gallery</h1>
<p>Generated by <code>generate_output_gallery.py</code> - rerun it after regenerating any config's output.</p>
{extra_body_html}
{render_gallery_grid(runs)}
</body>
</html>
"""


def main() -> None:
    runs = collect_runs()
    INDEX_FILE.write_text(build_html(runs), encoding="utf-8")
    print(f"Generated {INDEX_FILE} ({len(runs)} run(s))")


if __name__ == "__main__":
    main()
