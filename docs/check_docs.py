#!/usr/bin/env python3
"""Fast documentation drift checks that require only the standard library."""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[1]
PROFILE_DIR = ROOT / "ros2" / "robross_painter" / "config"
PROFILES = (
    "hardware_a4.yaml",
    "rviz_wall_a4.yaml",
    "rviz_taught_a4.yaml",
    "demo_v1_rviz.yaml",
)
SHARED_PROFILE_KEYS = (
    "tool_offset_xyz",
    "tool_offset_rpy",
    "tool_spin_deg",
    "claw_collision_size_xyz",
    "claw_collision_offset_xyz",
    "controller_sample_dt",
)
LINK_RE = re.compile(r"\[[^\]]*\]\(([^)]+)\)")
YAML_VALUE_RE = re.compile(r"^\s*([A-Za-z0-9_]+):\s*([^#]+?)\s*(?:#.*)?$")


def active_markdown_files() -> list[Path]:
    excluded = {
        ".agents",
        ".claude",
        ".codex",
        ".git",
        ".serena",
        "output",
        "build",
        "install",
        "log",
    }
    return [
        path
        for path in ROOT.rglob("*.md")
        if not any(part in excluded or part == "archive" for part in path.parts)
    ]


def check_links(path: Path, text: str) -> list[str]:
    errors = []
    for raw_target in LINK_RE.findall(text):
        target = raw_target.strip().strip("<>").split(maxsplit=1)[0]
        if target.startswith(("#", "http://", "https://", "mailto:")):
            continue
        target = unquote(target.split("#", 1)[0].split("?", 1)[0])
        if target and not (path.parent / target).resolve().exists():
            errors.append(f"{path.relative_to(ROOT)}: broken link: {raw_target}")
    return errors


def parse_profile(path: Path) -> dict[str, object]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        match = YAML_VALUE_RE.match(line)
        if not match or match.group(1) not in SHARED_PROFILE_KEYS:
            continue
        raw = match.group(2).strip()
        try:
            values[match.group(1)] = ast.literal_eval(raw)
        except (SyntaxError, ValueError):
            values[match.group(1)] = raw
    return values


def check_profiles() -> list[str]:
    errors = []
    parsed = {name: parse_profile(PROFILE_DIR / name) for name in PROFILES}
    baseline = parsed[PROFILES[0]]
    for key in SHARED_PROFILE_KEYS:
        if key not in baseline:
            errors.append(f"{PROFILES[0]}: missing {key}")
            continue
        for name in PROFILES[1:]:
            if parsed[name].get(key) != baseline[key]:
                errors.append(
                    f"profile drift: {name} {key}={parsed[name].get(key)!r}; "
                    f"expected {baseline[key]!r}"
                )
    return errors


def main() -> int:
    errors = []
    for path in active_markdown_files():
        text = path.read_text(encoding="utf-8")
        errors.extend(check_links(path, text))
        relative = path.relative_to(ROOT)
        if "handeye_calibration/" in text and relative != Path("CLAUDE.md"):
            errors.append(f"{relative}: references retired handeye_calibration/")
        if "output/curve_test_paths.json" in text:
            errors.append(
                f"{relative}: use output/demo_v1_a4_pen/curve_test_paths.json"
            )
        if "output/curve_test_preview.svg" in text:
            errors.append(
                f"{relative}: use output/demo_v1_a4_pen/curve_test_preview.svg"
            )

    errors.extend(check_profiles())
    if errors:
        print("Documentation checks failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("Documentation checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
