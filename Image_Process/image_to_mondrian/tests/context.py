"""Shared test setup: make the image_to_mondrian module importable and expose repo paths.

The scripts live directly under Image_Process/image_to_mondrian/ and are not
a package (they import each other as top-level modules), so tests add that
directory to sys.path the same way running a script from the repo root does.
"""

import sys
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = MODULE_DIR.parents[1]
SCRIPTS_DIR = MODULE_DIR
CONFIGS_DIR = REPO_ROOT / "configs"
FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
