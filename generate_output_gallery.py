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


def collect_runs() -> list:
    runs = []
    for config_path in sorted(CONFIGS_DIR.glob("*.json")):
        name = config_path.stem
        run_dir = OUTPUT_DIR / name
        if not run_dir.is_dir():
            continue

        config = json.loads(config_path.read_text(encoding="utf-8"))
        files = resolve_output_files(config["output"])
        if "paths_json" not in files:
            continue
        paths_json_path = run_dir / files["paths_json"]
        if not paths_json_path.is_file():
            continue

        painting_paths = json.loads(paths_json_path.read_text(encoding="utf-8"))
        runs.append(
            {
                "name": name,
                "run_dir": run_dir,
                "config": config,
                "files": files,
                "painting_paths": painting_paths,
            }
        )
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


def render_card(run: dict) -> str:
    name = run["name"]
    run_dir = run["run_dir"]
    files = run["files"]
    painting_paths = run["painting_paths"]
    canvas = painting_paths.get("canvas", {})
    style = painting_paths.get("style", "?")
    validation = painting_paths.get("validation", {"passed": None, "errors": []})
    debug = painting_paths.get("debug", {})

    rel = lambda filename: f"{name}/{filename}"

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

    return f"""
<section class="card">
  <h2>{escape(name)} <span class="style-badge">{escape(style)}</span></h2>
  <p class="canvas">{escape(str(canvas.get('width_mm', '?')))}mm x
     {escape(str(canvas.get('height_mm', '?')))}mm</p>
  {render_validation_badge(validation)}
  <div class="previews">{preview_html}</div>
  {render_stats_table(debug)}
  <p class="links">{' | '.join(links)}</p>
  <p class="run-dir">output/{escape(name)}/</p>
</section>
"""


STYLE = """
body { font-family: -apple-system, sans-serif; background: #f4f4f4; margin: 0; padding: 2rem; }
h1 { margin-top: 0; }
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
"""


def build_html(runs: list) -> str:
    cards = "\n".join(render_card(run) for run in runs)
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
<div class="grid">
{cards}
</div>
</body>
</html>
"""


def main() -> None:
    runs = collect_runs()
    INDEX_FILE.write_text(build_html(runs), encoding="utf-8")
    print(f"Generated {INDEX_FILE} ({len(runs)} run(s))")


if __name__ == "__main__":
    main()
