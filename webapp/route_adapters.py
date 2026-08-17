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
import math
import random
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
OUTPUT_DIR = REPO_ROOT / "output"
DEFAULT_SKETCH_DETAIL_MAX_PATHS = 400
DEFAULT_SKETCH_QWEN_MODEL = "qwen-image-2.0-pro"
DEFAULT_SKETCH_LINE_ART_THRESHOLD = 210
DEFAULT_SKETCH_OUTLINE_FLOOR_MM = 1.0
DEFAULT_SKETCH_SMOOTHING_RADIUS_MM = 0.5
DEFAULT_SKETCH_CONTINUOUS_PATHS = True
DEFAULT_PAPER_SIZE = "a4"
PAPER_SIZES = {
    "a4": "A4 (210 x 297 mm)",
    "4k_landscape": "4K landscape (525 x 375 mm)",
}

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
        "paper_templates": {
            "a4": "configs/sketch_demo_a4.json",
            "4k_landscape": "configs/sketch_demo_4k_landscape.json",
        },
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
        "paper_templates": {
            "a4": "configs/line_art_demo_a4.json",
            "4k_landscape": "configs/line_art_demo_4k_landscape.json",
        },
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


def _validate_sketch_settings(
    *,
    line_art_threshold=None,
    outline_min_path_length_mm=None,
    smoothing_radius_mm=None,
    continuous_paths=None,
    detail_max_paths=None,
):
    if line_art_threshold is not None and (
        not isinstance(line_art_threshold, int)
        or isinstance(line_art_threshold, bool)
        or not 0 <= line_art_threshold <= 255
    ):
        raise ValueError("Line-art threshold must be an integer from 0 to 255.")
    if outline_min_path_length_mm is not None and (
        isinstance(outline_min_path_length_mm, bool)
        or not isinstance(outline_min_path_length_mm, (int, float))
        or not math.isfinite(outline_min_path_length_mm)
        or outline_min_path_length_mm < 0
    ):
        raise ValueError("Semantic outline floor must be a non-negative number.")
    if smoothing_radius_mm is not None and (
        isinstance(smoothing_radius_mm, bool)
        or not isinstance(smoothing_radius_mm, (int, float))
        or not math.isfinite(smoothing_radius_mm)
        or not 0 <= smoothing_radius_mm <= 2
    ):
        raise ValueError("Smoothing radius must be a number from 0 to 2 mm.")
    if continuous_paths is not None and not isinstance(continuous_paths, bool):
        raise ValueError("Continuous paths must be a boolean.")
    if detail_max_paths is not None and (
        not isinstance(detail_max_paths, int)
        or isinstance(detail_max_paths, bool)
        or detail_max_paths < 0
    ):
        raise ValueError("Maximum detail paths must be a non-negative integer.")


def _resolve_inside(base: Path, candidate: Path, description: str) -> Path:
    base = base.resolve()
    resolved = candidate.resolve()
    if resolved == base or base not in resolved.parents:
        raise ValueError(f"{description} must stay inside {base}.")
    return resolved


def run_route(
    route_key: str,
    uploaded_file=None,
    *,
    semantic_preprocessing_enabled=None,
    qwen_model=None,
    line_art_threshold=None,
    outline_min_path_length_mm=None,
    smoothing_radius_mm=None,
    continuous_paths=None,
    detail_max_paths=None,
    paper_size=DEFAULT_PAPER_SIZE,
) -> str:
    """uploaded_file is a Werkzeug FileStorage (or None for routes that
    don't need one). Returns the new run's name (output/<name>/)."""
    adapter = ROUTE_ADAPTERS[route_key]
    paper_templates = adapter.get("paper_templates")
    if paper_templates is not None:
        if paper_size not in paper_templates:
            raise ValueError(f"Unsupported paper size for {route_key}: {paper_size!r}")
        template_config = paper_templates[paper_size]
    else:
        if paper_size != DEFAULT_PAPER_SIZE:
            raise ValueError(f"Route {route_key!r} only supports A4 paper.")
        template_config = adapter["template_config"]
    if route_key == "sketch":
        _validate_sketch_settings(
            line_art_threshold=line_art_threshold,
            outline_min_path_length_mm=outline_min_path_length_mm,
            smoothing_radius_mm=smoothing_radius_mm,
            continuous_paths=continuous_paths,
            detail_max_paths=detail_max_paths,
        )
    if route_key == "sketch" and qwen_model is not None:
        if not isinstance(qwen_model, str) or not qwen_model.strip():
            raise ValueError("Qwen model must be a non-empty string.")
        qwen_model = qwen_model.strip()
    run_name = f"upload_{int(time.time())}_{route_key}"
    run_dir = OUTPUT_DIR / run_name
    run_dir.mkdir(parents=True)

    template_path = REPO_ROOT / template_config
    config = json.loads(template_path.read_text(encoding="utf-8"))
    config = copy.deepcopy(config)
    config["output"]["directory"] = f"output/{run_name}"

    if route_key == "sketch" and semantic_preprocessing_enabled is not None:
        config.setdefault("semantic_preprocessing", {})["enabled"] = bool(
            semantic_preprocessing_enabled
        )
    if route_key == "sketch" and qwen_model is not None:
        config.setdefault("semantic_preprocessing", {})["model"] = qwen_model
    if route_key == "sketch" and line_art_threshold is not None:
        config.setdefault("outline_extraction", {})[
            "line_art_threshold"
        ] = line_art_threshold
    if route_key == "sketch" and outline_min_path_length_mm is not None:
        config.setdefault("outline_extraction", {})[
            "min_path_length_mm"
        ] = float(outline_min_path_length_mm)
    if route_key == "sketch" and smoothing_radius_mm is not None:
        config.setdefault("path_generation", {})[
            "smoothing_radius_mm"
        ] = float(smoothing_radius_mm)
    if route_key == "sketch" and continuous_paths is not None:
        config.setdefault("path_generation", {})["continuous_paths"] = continuous_paths
    if route_key == "sketch" and detail_max_paths is not None:
        config.setdefault("detail_extraction", {})["max_paths"] = detail_max_paths

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


def retrace_sketch_run(
    source_run_name: str,
    *,
    line_art_threshold: int,
    outline_min_path_length_mm: float,
    smoothing_radius_mm: float,
    continuous_paths: bool,
    detail_max_paths: int,
) -> str:
    """Create a new sketch run from a saved semantic intermediate.

    Every input asset is resolved beneath the source run, copied to the new
    run, and passed to the generator's explicit no-network reuse mode.
    """
    _validate_sketch_settings(
        line_art_threshold=line_art_threshold,
        outline_min_path_length_mm=outline_min_path_length_mm,
        smoothing_radius_mm=smoothing_radius_mm,
        continuous_paths=continuous_paths,
        detail_max_paths=detail_max_paths,
    )
    if not source_run_name or any(
        character not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._-"
        for character in source_run_name
    ) or ".." in source_run_name:
        raise ValueError(f"Invalid run name: {source_run_name!r}")

    output_dir = OUTPUT_DIR.resolve()
    source_run_dir = _resolve_inside(
        output_dir, OUTPUT_DIR / source_run_name, "Source run"
    )
    if not source_run_dir.is_dir():
        raise ValueError(f"No such run: {source_run_name!r}")
    config_path = _resolve_inside(
        source_run_dir, source_run_dir / "_config.json", "Run config"
    )
    if not config_path.is_file():
        raise ValueError("Retracing requires the run's saved _config.json.")
    try:
        config = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        raise ValueError("The saved run config is unreadable or invalid.") from None
    if config.get("style") != "sketch":
        raise ValueError("Only sketch runs can be retraced.")

    paths_name = config.get("output", {}).get("painting_paths_file")
    if not isinstance(paths_name, str) or not paths_name:
        raise ValueError("The saved sketch config has no painting-paths file.")
    paths_file = _resolve_inside(
        source_run_dir, source_run_dir / paths_name, "Painting paths"
    )
    try:
        source_paths = json.loads(paths_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        raise ValueError("The saved painting paths are missing or unreadable.") from None
    if source_paths.get("style") != "sketch":
        raise ValueError("Only sketch runs can be retraced.")
    semantic_status = source_paths.get("debug", {}).get(
        "semantic_preprocessing_status"
    )
    if semantic_status not in ("used", "reused"):
        raise ValueError(
            "Retracing requires a successfully generated or reused Qwen image."
        )

    source_value = config.get("source_image", {}).get("path")
    if not isinstance(source_value, str) or not source_value:
        raise ValueError("The saved sketch config has no source photo.")
    source_photo = Path(source_value)
    if not source_photo.is_absolute():
        source_photo = REPO_ROOT / source_photo
    source_photo = _resolve_inside(source_run_dir, source_photo, "Source photo")
    if not source_photo.is_file():
        raise ValueError("The source photo for this sketch run is missing.")
    try:
        with source_photo.open("rb") as source_file:
            if not source_file.read(1):
                raise OSError
    except OSError:
        raise ValueError("The source photo is unreadable or empty.") from None

    semantic_name = config.get("output", {}).get("semantic_outline_image_file")
    if not isinstance(semantic_name, str) or not semantic_name:
        raise ValueError("The saved sketch config has no semantic image file.")
    semantic_image = _resolve_inside(
        source_run_dir, source_run_dir / semantic_name, "Saved semantic image"
    )
    if not semantic_image.is_file():
        raise ValueError("The saved semantic image for this sketch run is missing.")
    try:
        if not semantic_image.read_bytes():
            raise OSError
    except OSError:
        raise ValueError("The saved semantic image is unreadable or empty.") from None

    timestamp = int(time.time())
    while (OUTPUT_DIR / f"retrace_{timestamp}_sketch").exists():
        timestamp += 1
    run_name = f"retrace_{timestamp}_sketch"
    run_dir = OUTPUT_DIR / run_name
    run_dir.mkdir(parents=True)
    try:
        # Fixed role-based names prevent a malicious or malformed source config
        # from making the two copied assets overwrite each other or _config.json.
        photo_copy = run_dir / f"source_photo{source_photo.suffix}"
        semantic_copy = run_dir / f"saved_semantic{semantic_image.suffix}"
        shutil.copy2(source_photo, photo_copy)
        shutil.copy2(semantic_image, semantic_copy)

        new_config = copy.deepcopy(config)
        new_config["profile_name"] = run_name
        new_config["source_image"]["path"] = str(
            photo_copy.relative_to(REPO_ROOT)
        )
        new_config["output"]["directory"] = str(run_dir.relative_to(REPO_ROOT))
        new_config["output"]["semantic_outline_image_file"] = semantic_copy.name
        new_config.setdefault("semantic_preprocessing", {})["enabled"] = True
        new_config.setdefault("outline_extraction", {})[
            "line_art_threshold"
        ] = line_art_threshold
        new_config["outline_extraction"]["min_path_length_mm"] = float(
            outline_min_path_length_mm
        )
        new_config.setdefault("path_generation", {})["smoothing_radius_mm"] = float(
            smoothing_radius_mm
        )
        new_config["path_generation"]["continuous_paths"] = continuous_paths
        new_config.setdefault("detail_extraction", {})["max_paths"] = detail_max_paths
        new_config["retrace_provenance"] = {
            "source_run": source_run_name,
            "source_config": str(config_path.relative_to(REPO_ROOT)),
            "saved_semantic_image": str(semantic_image.relative_to(REPO_ROOT)),
        }
        new_config_path = run_dir / "_config.json"
        new_config_path.write_text(
            json.dumps(new_config, indent=2), encoding="utf-8"
        )

        script = "Image_Process/sketch/generate_sketch_paths.py"
        try:
            result = subprocess.run(
                [
                    sys.executable,
                    script,
                    "--config",
                    str(new_config_path.relative_to(REPO_ROOT)),
                    "--saved-semantic-image",
                    str(semantic_copy.relative_to(REPO_ROOT)),
                ],
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
            )
        except OSError as error:
            raise RouteRunError(script, -1, str(error)) from None
        if result.returncode != 0:
            raise RouteRunError(script, result.returncode, result.stderr)
    except Exception:
        shutil.rmtree(run_dir, ignore_errors=True)
        raise
    return run_name
