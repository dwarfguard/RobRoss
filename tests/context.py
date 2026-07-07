"""Shared test setup: make scripts/ importable and expose repo paths.

The scripts/ directory is not a package (the scripts import each other as
top-level modules), so tests add it to sys.path the same way running a
script from the repo root does.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
CONFIGS_DIR = REPO_ROOT / "configs"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))
