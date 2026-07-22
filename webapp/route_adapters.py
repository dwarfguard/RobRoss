"""Registry of the pipeline routes the webapp can drive, plus the
orchestration that turns "route + optional uploaded photo" into a real
`output/<run>/` directory full of generated artifacts.

Each route's generator scripts import their own sibling modules
(`config_loader`, `path_validation`, `path_ordering`, ...) as bare top-level
names, not a proper package - every route folder has its own same-named
copy (see CLAUDE.md's "Key architectural constraints" section). Importing
more than one route's modules into this single long-running process would
collide in sys.modules, so every route is invoked as a subprocess running its
existing CLI exactly the way the docs already describe
(`python3 Image_Process/<route>/generate_*.py --config <path>`), never
imported directly.
"""

import copy
import json
import random
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
OUTPUT_DIR = REPO_ROOT / "output"

# To add a new route later: add one entry here. Nothing else in this file or
# in app.py needs to change.
ROUTE_ADAPTERS = {
    "image_to_mondrian": {
        "label": "Photo -> quantized Mondrian fill",
        "template_config": "configs/image_to_mondrian_demo_a4.json",
        "needs_source_image": True,
        "steps": [("Image_Process/image_to_mondrian/generate_painting_paths.py", [])],
    },
    "sketch": {
        "label": "Photo -> traced outline",
        "template_config": "configs/sketch_demo_a4.json",
        "needs_source_image": True,
        "steps": [("Image_Process/sketch/generate_sketch_paths.py", [])],
    },
    "mondrian": {
        "label": "Procedural Mondrian (random layout, no photo needed)",
        "template_config": "configs/demo_v1_a4_pen.json",
        "needs_source_image": False,
        "steps": [
            ("Image_Process/mondrian/mondrian_generator.py", ["--seed", "{seed}"]),
            ("Image_Process/mondrian/generate_painting_paths.py", []),
        ],
    },
    "gemini_mondrian": {
        "label": "Photo -> Gemini Mondrian style -> quantized fill",
        "template_config": "configs/gemini_mondrian_demo_a4.json",
        "needs_source_image": True,
        "steps": [("Image_Process/gemini_mondrian/generate_painting_paths.py", [])],
    },
    "line_art": {
        "label": "Clean line art / technical illustration -> traced outline",
        "template_config": "configs/line_art_demo_a4.json",
        "needs_source_image": True,
        "steps": [("Image_Process/line_art/generate_line_art_paths.py", [])],
    },
}


class RouteRunError(Exception):
    """Raised when a route's subprocess step fails. Carries the step's
    stderr so the caller can show the real error instead of a generic 500."""

    def __init__(self, script: str, returncode: int, stderr: str):
        super().__init__(f"{script} exited {returncode}")
        self.script = script
        self.returncode = returncode
        self.stderr = stderr


def _slugify(filename: str) -> str:
    keep = [c if c.isalnum() or c in "._-" else "_" for c in filename]
    return "".join(keep) or "upload"


def run_route(route_key: str, uploaded_file=None) -> str:
    """uploaded_file is a Werkzeug FileStorage (or None for routes that
    don't need one). Returns the new run's name (output/<name>/)."""
    adapter = ROUTE_ADAPTERS[route_key]
    run_name = f"upload_{int(time.time())}_{route_key}"
    run_dir = OUTPUT_DIR / run_name
    run_dir.mkdir(parents=True)

    template_path = REPO_ROOT / adapter["template_config"]
    config = json.loads(template_path.read_text(encoding="utf-8"))
    config = copy.deepcopy(config)
    config["output"]["directory"] = f"output/{run_name}"

    if adapter["needs_source_image"]:
        if uploaded_file is None or not uploaded_file.filename:
            raise ValueError(f"route '{route_key}' requires an uploaded image")
        saved_name = f"upload_{_slugify(uploaded_file.filename)}"
        saved_path = run_dir / saved_name
        uploaded_file.save(saved_path)
        config["source_image"]["path"] = f"output/{run_name}/{saved_name}"

    config_path = run_dir / "_config.json"
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")

    seed = str(random.randint(1, 1_000_000))
    for script, extra_args in adapter["steps"]:
        args = [
            sys.executable,  # same interpreter running app.py, not a bare "python3" off PATH
            script,
            "--config",
            str(config_path.relative_to(REPO_ROOT)),
        ] + [arg.format(seed=seed) for arg in extra_args]
        result = subprocess.run(
            args, cwd=REPO_ROOT, capture_output=True, text=True
        )
        if result.returncode != 0:
            raise RouteRunError(script, result.returncode, result.stderr)

    return run_name
