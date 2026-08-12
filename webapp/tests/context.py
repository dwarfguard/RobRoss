"""Make the repository webapp modules importable in direct unittest runs."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
WEBAPP_DIR = REPO_ROOT / "webapp"
for path in (REPO_ROOT, WEBAPP_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
