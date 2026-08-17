#!/usr/bin/env python3
"""Small local control panel: upload a photo, pick a pipeline route, click
process, browse the result - built on top of generate_output_gallery.py's
existing card rendering (reused as-is, not reimplemented) plus
route_adapters.py's subprocess orchestration of the existing per-route CLI
scripts.

Run from the repo root:

    pip install flask
    python3 webapp/app.py

Then open http://127.0.0.1:5050 . Local dev tool only - no auth, binds to
127.0.0.1 (localhost) on purpose. Do not change the host to 0.0.0.0 or expose
this to a network.
"""

import math
import re
import shutil
import sys
from html import escape
from pathlib import Path

from flask import Flask, redirect, request, send_from_directory, url_for

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))  # to import generate_output_gallery below

import generate_output_gallery  # noqa: E402
import route_adapters  # noqa: E402

app = Flask(__name__)


def render_upload_form() -> str:
    options = "\n".join(
        f'<option value="{escape(key)}" data-needs-image="{"true" if adapter["needs_source_image"] else "false"}">'
        f'{escape(adapter["label"])}</option>'
        for key, adapter in route_adapters.ROUTE_ADAPTERS.items()
    )
    paper_options = "\n".join(
        f'<option value="{escape(key)}">{escape(label)}</option>'
        for key, label in route_adapters.PAPER_SIZES.items()
    )
    return f"""
<section class="upload-form">
  <form method="post" action="/process" enctype="multipart/form-data">
    <label>Route:
      <select name="route" id="route-select">{options}</select>
    </label>
    <label id="image-label">Photo:
      <input type="file" name="image" id="image-input" accept="image/*">
    </label>
    <label id="paper-size-label">Paper size:
      <select name="paper_size" id="paper-size-select">{paper_options}</select>
    </label>
    <label id="semantic-label" class="checkbox-label">
      <input type="checkbox" name="enhance_overlapping_forms"
             id="semantic-input" checked>
      Enhance overlapping forms with Qwen
    </label>
    <label id="qwen-model-label">Qwen image model:
      <input type="text" name="qwen_model" id="qwen-model-input"
             value="{route_adapters.DEFAULT_SKETCH_QWEN_MODEL}" required>
    </label>
    <label id="detail-max-paths-label">Maximum detail paths:
      <input type="number" name="maximum_detail_paths"
             id="detail-max-paths-input"
             value="{route_adapters.DEFAULT_SKETCH_DETAIL_MAX_PATHS}"
             min="0" step="1" inputmode="numeric">
    </label>
    <label id="line-art-threshold-label">Line-art threshold:
      <input type="number" name="line_art_threshold"
             id="line-art-threshold-input"
             value="{route_adapters.DEFAULT_SKETCH_LINE_ART_THRESHOLD}"
             min="0" max="255" step="1" inputmode="numeric">
    </label>
    <label id="outline-floor-label">Semantic outline floor (mm):
      <input type="number" name="semantic_outline_floor_mm"
             id="outline-floor-input"
             value="{route_adapters.DEFAULT_SKETCH_OUTLINE_FLOOR_MM}"
             min="0" step="0.1">
    </label>
    <label id="smoothing-radius-label">Smoothing radius (mm):
      <input type="number" name="smoothing_radius_mm"
             id="smoothing-radius-input"
             value="{route_adapters.DEFAULT_SKETCH_SMOOTHING_RADIUS_MM}"
             min="0" max="2" step="0.1">
    </label>
    <label id="continuous-paths-label" class="checkbox-label">
      <input type="checkbox" name="continuous_paths"
             id="continuous-paths-input" checked>
      Use continuous paths
    </label>
    <button type="submit">Process</button>
  </form>
</section>
<script>
  const routeSelect = document.getElementById('route-select');
  const imageLabel = document.getElementById('image-label');
  const imageInput = document.getElementById('image-input');
  const paperSizeLabel = document.getElementById('paper-size-label');
  const paperSizeSelect = document.getElementById('paper-size-select');
  const semanticLabel = document.getElementById('semantic-label');
  const semanticInput = document.getElementById('semantic-input');
  const qwenModelLabel = document.getElementById('qwen-model-label');
  const qwenModelInput = document.getElementById('qwen-model-input');
  const detailMaxPathsLabel = document.getElementById('detail-max-paths-label');
  const detailMaxPathsInput = document.getElementById('detail-max-paths-input');
  const lineArtThresholdLabel = document.getElementById('line-art-threshold-label');
  const lineArtThresholdInput = document.getElementById('line-art-threshold-input');
  const outlineFloorLabel = document.getElementById('outline-floor-label');
  const outlineFloorInput = document.getElementById('outline-floor-input');
  const smoothingRadiusLabel = document.getElementById('smoothing-radius-label');
  const smoothingRadiusInput = document.getElementById('smoothing-radius-input');
  const continuousPathsLabel = document.getElementById('continuous-paths-label');
  const continuousPathsInput = document.getElementById('continuous-paths-input');
  function syncImageField() {{
    const opt = routeSelect.options[routeSelect.selectedIndex];
    const needsImage = opt.dataset.needsImage === 'true';
    imageLabel.style.display = needsImage ? '' : 'none';
    imageInput.disabled = !needsImage;
    imageInput.required = needsImage;
    const supportsPaperSize = routeSelect.value === 'sketch' || routeSelect.value === 'line_art';
    paperSizeLabel.style.display = supportsPaperSize ? '' : 'none';
    paperSizeSelect.disabled = !supportsPaperSize;
    const supportsSemantic = routeSelect.value === 'sketch';
    semanticLabel.style.display = supportsSemantic ? '' : 'none';
    semanticInput.disabled = !supportsSemantic;
    qwenModelLabel.style.display = supportsSemantic ? '' : 'none';
    qwenModelInput.disabled = !supportsSemantic;
    detailMaxPathsLabel.style.display = supportsSemantic ? '' : 'none';
    detailMaxPathsInput.disabled = !supportsSemantic;
    lineArtThresholdLabel.style.display = supportsSemantic ? '' : 'none';
    lineArtThresholdInput.disabled = !supportsSemantic;
    outlineFloorLabel.style.display = supportsSemantic ? '' : 'none';
    outlineFloorInput.disabled = !supportsSemantic;
    smoothingRadiusLabel.style.display = supportsSemantic ? '' : 'none';
    smoothingRadiusInput.disabled = !supportsSemantic;
    continuousPathsLabel.style.display = supportsSemantic ? '' : 'none';
    continuousPathsInput.disabled = !supportsSemantic;
  }}
  routeSelect.addEventListener('change', syncImageField);
  syncImageField();
</script>
"""


EXTRA_STYLE = """
.upload-form { background: white; border-radius: 8px; padding: 1rem 1.25rem; margin-bottom: 1.5rem; box-shadow: 0 1px 3px rgba(0,0,0,0.15); }
.upload-form form { display: flex; gap: 1rem; align-items: center; flex-wrap: wrap; }
.upload-form label { display: flex; flex-direction: column; font-size: 0.85rem; color: #555; gap: 0.25rem; }
.upload-form .checkbox-label { flex-direction: row; align-items: center; font-size: 0.9rem; color: #333; }
.upload-form button { padding: 0.5rem 1rem; border: none; border-radius: 4px; background: #2563eb; color: white; cursor: pointer; }
.error-box { background: #fff0f0; border: 1px solid #f8d6d6; border-radius: 8px; padding: 1rem; margin-bottom: 1.5rem; white-space: pre-wrap; font-family: monospace; font-size: 0.85rem; }
"""


@app.route("/")
def index():
    runs = generate_output_gallery.collect_runs()
    body = render_upload_form() + generate_output_gallery.render_gallery_grid(
        runs, base_url="/output/", show_delete=True
    )
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>RobRoss control panel</title>
<style>{generate_output_gallery.STYLE}{EXTRA_STYLE}</style>
</head>
<body>
<h1>RobRoss control panel</h1>
<p>Upload a photo, pick a route, click Process. Runs land in <code>output/&lt;name&gt;/</code>.</p>
{body}
</body>
</html>
"""


@app.route("/process", methods=["POST"])
def process():
    route_key = request.form.get("route")
    if route_key not in route_adapters.ROUTE_ADAPTERS:
        return render_error(f"Unknown route: {route_key!r}")

    uploaded_file = request.files.get("image")
    try:
        paper_size = request.form.get(
            "paper_size", route_adapters.DEFAULT_PAPER_SIZE
        )
        semantic_enabled = None
        qwen_model = None
        line_art_threshold = None
        outline_floor = None
        smoothing_radius = None
        continuous_paths = None
        detail_max_paths = None
        if route_key == "sketch":
            semantic_enabled = request.form.get("enhance_overlapping_forms") == "on"
            qwen_model = request.form.get(
                "qwen_model", route_adapters.DEFAULT_SKETCH_QWEN_MODEL
            ).strip()
            if not qwen_model:
                raise ValueError("Qwen model must be a non-empty string.")
            raw_threshold = request.form.get(
                "line_art_threshold",
                str(route_adapters.DEFAULT_SKETCH_LINE_ART_THRESHOLD),
            ).strip()
            if not re.fullmatch(r"[0-9]+", raw_threshold):
                raise ValueError("Line-art threshold must be an integer from 0 to 255.")
            line_art_threshold = int(raw_threshold)
            if not 0 <= line_art_threshold <= 255:
                raise ValueError("Line-art threshold must be an integer from 0 to 255.")
            try:
                outline_floor = float(
                    request.form.get(
                        "semantic_outline_floor_mm",
                        str(route_adapters.DEFAULT_SKETCH_OUTLINE_FLOOR_MM),
                    ).strip()
                )
            except ValueError:
                raise ValueError(
                    "Semantic outline floor must be a non-negative number."
                ) from None
            if not math.isfinite(outline_floor) or outline_floor < 0:
                raise ValueError("Semantic outline floor must be a non-negative number.")
            try:
                smoothing_radius = float(
                    request.form.get(
                        "smoothing_radius_mm",
                        str(route_adapters.DEFAULT_SKETCH_SMOOTHING_RADIUS_MM),
                    ).strip()
                )
            except ValueError:
                raise ValueError("Smoothing radius must be from 0 to 2 mm.") from None
            if not math.isfinite(smoothing_radius) or not 0 <= smoothing_radius <= 2:
                raise ValueError("Smoothing radius must be from 0 to 2 mm.")
            continuous_paths = request.form.get("continuous_paths") == "on"
            raw_detail_max_paths = request.form.get(
                "maximum_detail_paths",
                str(route_adapters.DEFAULT_SKETCH_DETAIL_MAX_PATHS),
            ).strip()
            if not re.fullmatch(r"[0-9]+", raw_detail_max_paths):
                raise ValueError(
                    "Maximum detail paths must be a non-negative integer."
                )
            detail_max_paths = int(raw_detail_max_paths)
        route_adapters.run_route(
            route_key,
            uploaded_file,
            semantic_preprocessing_enabled=semantic_enabled,
            qwen_model=qwen_model,
            line_art_threshold=line_art_threshold,
            outline_min_path_length_mm=outline_floor,
            smoothing_radius_mm=smoothing_radius,
            continuous_paths=continuous_paths,
            detail_max_paths=detail_max_paths,
            paper_size=paper_size,
        )
    except route_adapters.RouteRunError as error:
        return render_error(
            f"{error.script} failed (exit {error.returncode}):\n\n{error.stderr}"
        )
    except ValueError as error:
        return render_error(str(error))

    return redirect(url_for("index"), code=303)


def render_error(message: str) -> str:
    return f"""<!doctype html>
<html lang="en">
<head><meta charset="utf-8"><title>Processing failed</title>
<style>{generate_output_gallery.STYLE}{EXTRA_STYLE}</style></head>
<body>
<h1>Processing failed</h1>
<div class="error-box">{escape(message)}</div>
<p><a href="/">Back</a></p>
</body>
</html>
""", 400


def _parse_retrace_settings():
    raw_threshold = request.form.get("line_art_threshold", "").strip()
    if not re.fullmatch(r"[0-9]+", raw_threshold):
        raise ValueError("Line-art threshold must be an integer from 0 to 255.")
    threshold = int(raw_threshold)
    if not 0 <= threshold <= 255:
        raise ValueError("Line-art threshold must be an integer from 0 to 255.")
    try:
        outline_floor = float(
            request.form.get("semantic_outline_floor_mm", "").strip()
        )
    except ValueError:
        raise ValueError("Semantic outline floor must be a non-negative number.") from None
    if not math.isfinite(outline_floor) or outline_floor < 0:
        raise ValueError("Semantic outline floor must be a non-negative number.")
    try:
        smoothing_radius = float(request.form.get("smoothing_radius_mm", "").strip())
    except ValueError:
        raise ValueError("Smoothing radius must be from 0 to 2 mm.") from None
    if not math.isfinite(smoothing_radius) or not 0 <= smoothing_radius <= 2:
        raise ValueError("Smoothing radius must be from 0 to 2 mm.")
    raw_detail_budget = request.form.get("maximum_detail_paths", "").strip()
    if not re.fullmatch(r"[0-9]+", raw_detail_budget):
        raise ValueError("Maximum detail paths must be a non-negative integer.")
    return {
        "line_art_threshold": threshold,
        "outline_min_path_length_mm": outline_floor,
        "smoothing_radius_mm": smoothing_radius,
        "continuous_paths": request.form.get("continuous_paths") == "on",
        "detail_max_paths": int(raw_detail_budget),
    }


@app.route("/output/<name>/retrace", methods=["POST"])
def retrace_run(name):
    try:
        settings = _parse_retrace_settings()
        route_adapters.retrace_sketch_run(name, **settings)
        generate_output_gallery.main()
    except route_adapters.RouteRunError as error:
        return render_error(
            f"{error.script} failed (exit {error.returncode}):\n\n{error.stderr}"
        )
    except ValueError as error:
        return render_error(str(error))
    return redirect(url_for("index"), code=303)


@app.route("/output/<path:filename>")
def output_files(filename):
    return send_from_directory(generate_output_gallery.OUTPUT_DIR, filename)


_RUN_NAME_RE = re.compile(r"^[A-Za-z0-9._-]+$")


@app.route("/output/<name>/delete", methods=["POST"])
def delete_run(name):
    """Deletes a whole output/<name>/ run directory - the smallest unit
    collect_runs() treats as coherent (a run's painting_paths.json + its
    preview files belong together; deleting just one would leave the rest
    orphaned and broken in the gallery). Two independent checks against
    path traversal: a charset allowlist on the raw name (same convention as
    route_adapters._slugify()) before it ever touches a path, and an
    is_relative_to() containment check on the resolved path after."""
    if not _RUN_NAME_RE.match(name) or ".." in name:
        return render_error(f"Invalid run name: {name!r}")

    output_dir = generate_output_gallery.OUTPUT_DIR.resolve()
    run_dir = (output_dir / name).resolve()
    if run_dir == output_dir or output_dir not in run_dir.parents:
        return render_error(f"Refusing to delete outside output/: {name!r}")
    if not run_dir.is_dir():
        return render_error(f"No such run: {name!r}")

    shutil.rmtree(run_dir)
    generate_output_gallery.main()  # keep the static output/index.html in sync too

    return redirect(url_for("index"), code=303)


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5050)
