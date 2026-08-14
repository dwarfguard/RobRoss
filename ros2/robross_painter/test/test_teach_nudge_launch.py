import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "teach_nudge.launch.py"
SPEC = importlib.util.spec_from_file_location("teach_nudge_launch", LAUNCH_FILE)
teach_launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(teach_launch)


def test_qualification_host_gate_accepts_matching_evidence(monkeypatch):
    modules = {
        "host_time_preflight": SimpleNamespace(
            validate_receipt=lambda path, session, **kwargs: ([], {"status": "pass"})
        ),
        "host_clock_monitor": SimpleNamespace(
            validate_monitor=lambda path, session: ([], {"status": "running"})
        ),
    }
    monkeypatch.setattr(teach_launch, "load_host_module", modules.__getitem__)

    teach_launch.validate_host_gate("session-a", "receipt.json", "monitor.json")


def test_qualification_host_gate_rejects_monitor_failure(monkeypatch):
    modules = {
        "host_time_preflight": SimpleNamespace(
            validate_receipt=lambda path, session, **kwargs: ([], {"status": "pass"})
        ),
        "host_clock_monitor": SimpleNamespace(
            validate_monitor=lambda path, session: (["monitor stopped"], None)
        ),
    }
    monkeypatch.setattr(teach_launch, "load_host_module", modules.__getitem__)

    with pytest.raises(RuntimeError, match="monitor stopped"):
        teach_launch.validate_host_gate(
            "session-a", "receipt.json", "monitor.json"
        )
