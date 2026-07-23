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
    return f"""
<section class="upload-form">
  <form method="post" action="/process" enctype="multipart/form-data">
    <label>Route:
      <select name="route" id="route-select">{options}</select>
    </label>
    <label id="image-label">Photo:
      <input type="file" name="image" id="image-input" accept="image/*">
    </label>
    <button type="submit">Process</button>
  </form>
</section>
<script>
  const routeSelect = document.getElementById('route-select');
  const imageLabel = document.getElementById('image-label');
  const imageInput = document.getElementById('image-input');
  function syncImageField() {{
    const opt = routeSelect.options[routeSelect.selectedIndex];
    const needsImage = opt.dataset.needsImage === 'true';
    imageLabel.style.display = needsImage ? '' : 'none';
    imageInput.disabled = !needsImage;
    imageInput.required = needsImage;
  }}
  routeSelect.addEventListener('change', syncImageField);
  syncImageField();
</script>
"""


EXTRA_STYLE = """
.upload-form { background: white; border-radius: 8px; padding: 1rem 1.25rem; margin-bottom: 1.5rem; box-shadow: 0 1px 3px rgba(0,0,0,0.15); }
.upload-form form { display: flex; gap: 1rem; align-items: center; flex-wrap: wrap; }
.upload-form label { display: flex; flex-direction: column; font-size: 0.85rem; color: #555; gap: 0.25rem; }
.upload-form button { padding: 0.5rem 1rem; border: none; border-radius: 4px; background: #2563eb; color: white; cursor: pointer; }
.error-box { background: #fff0f0; border: 1px solid #f8d6d6; border-radius: 8px; padding: 1rem; margin-bottom: 1.5rem; white-space: pre-wrap; font-family: monospace; font-size: 0.85rem; }
"""


@app.route("/")
def index():
    runs = generate_output_gallery.collect_runs()
    body = render_upload_form() + generate_output_gallery.render_gallery_grid(
        runs, base_url="/output/"
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
        route_adapters.run_route(route_key, uploaded_file)
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


@app.route("/output/<path:filename>")
def output_files(filename):
    return send_from_directory(generate_output_gallery.OUTPUT_DIR, filename)


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5050)
