"""Make the sketch route's direct-entrypoint modules importable in tests."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
MODULE_DIR = Path(__file__).resolve().parents[1]
CONFIGS_DIR = REPO_ROOT / "configs"

if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))
