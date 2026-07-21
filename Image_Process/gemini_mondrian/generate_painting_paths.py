#!/usr/bin/env python3
"""Photo -> Gemini image-to-image Mondrian-style transfer -> painting_paths.json.

Pipeline:

    Config profile (configs/gemini_mondrian_*.json)
      -> gemini_client.generate_styled_image()   photo -> Gemini-generated Mondrian-style image
      -> (clone downstream_template_config, point source_image.path at that image)
      -> subprocess: Image_Process/image_to_mondrian/generate_painting_paths.py
      -> output/<run>/painting_paths.json (+ preview SVG + quantized preview PNG)

The vectorization step (quantize/segment/fill/border-trace) is not
reimplemented here - it's the exact same image_to_mondrian pipeline every
other photo lands in, just fed a Gemini-generated image instead of the
original photo. Run as a subprocess, not imported, for the same reason
webapp/route_adapters.py never imports more than one route's modules into a
single process: config_loader/path_validation/etc. are same-named top-level
modules duplicated per route folder and would collide in sys.modules.

Run from the repo root:

    python3 Image_Process/gemini_mondrian/generate_painting_paths.py \\
        --config configs/gemini_mondrian_demo_a4.json
"""

import argparse
import copy
import json
import subprocess
import sys
from pathlib import Path

from config_loader import load_config
from gemini_client import generate_styled_image

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_FILE = "configs/gemini_mondrian_demo_a4.json"


def build_downstream_config(config: dict, gemini_image_path: Path, output_dir: Path) -> dict:
    """Clone downstream_template_config, pointed at the Gemini-generated
    image and this run's own output directory. Pure logic, no I/O or network
    calls - kept separate from main() so it's directly unit-testable."""
    template_path = REPO_ROOT / config["downstream_template_config"]
    downstream_config = json.loads(template_path.read_text(encoding="utf-8"))
    downstream_config = copy.deepcopy(downstream_config)

    downstream_config["source_image"]["path"] = str(
        gemini_image_path.relative_to(REPO_ROOT)
    )
    downstream_config["output"]["directory"] = str(output_dir.relative_to(REPO_ROOT))
    downstream_config["profile_name"] = (
        f"{downstream_config.get('profile_name', 'unknown')}_via_gemini"
    )

    return downstream_config


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Photo -> Gemini Mondrian-style transfer -> painting_paths.json (via image_to_mondrian)."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help=f"Path to a gemini_mondrian pipeline config JSON file (default: {DEFAULT_CONFIG_FILE}).",
    )
    args = parser.parse_args()

    config = load_config(args.config)

    output_dir = REPO_ROOT / config["output"]["directory"]
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Gemini: source photo -> Mondrian-style image.
    source_image_path = REPO_ROOT / config["source_image"]["path"]
    image_bytes = generate_styled_image(
        source_image_path, config["gemini"]["prompt"], config["gemini"]["model"]
    )
    gemini_image_path = output_dir / config["output"]["gemini_output_image_file"]
    gemini_image_path.write_bytes(image_bytes)
    print(f"Generated {gemini_image_path}")

    # 2. Keep this route's own config (prompt/model included) for provenance
    # - save it before the next step, which may write to the same path this
    # config was loaded from (when invoked via webapp/, --config and the
    # downstream config below both live at output/<run>/_config.json).
    gemini_config_path = output_dir / "_gemini_config.json"
    gemini_config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")

    # 3. Clone the downstream image_to_mondrian config, pointed at the
    # Gemini output image and this same output directory.
    downstream_config = build_downstream_config(config, gemini_image_path, output_dir)
    downstream_config_path = output_dir / "_config.json"
    downstream_config_path.write_text(
        json.dumps(downstream_config, indent=2), encoding="utf-8"
    )

    # 4. Hand off to the existing image_to_mondrian pipeline as a subprocess.
    result = subprocess.run(
        [
            sys.executable,
            "Image_Process/image_to_mondrian/generate_painting_paths.py",
            "--config",
            str(downstream_config_path.relative_to(REPO_ROOT)),
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    if result.stdout:
        print(result.stdout, end="")
    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        sys.exit(result.returncode)


if __name__ == "__main__":
    main()
