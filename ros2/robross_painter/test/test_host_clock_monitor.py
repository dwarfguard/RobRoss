import importlib.util
import json
import os
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE_ROOT / "scripts"
SPEC_TIME = importlib.util.spec_from_file_location(
    "host_time_preflight", SCRIPTS / "host_time_preflight.py"
)
host_time = importlib.util.module_from_spec(SPEC_TIME)
SPEC_TIME.loader.exec_module(host_time)
import sys
sys.modules["host_time_preflight"] = host_time
SPEC = importlib.util.spec_from_file_location(
    "host_clock_monitor", SCRIPTS / "host_clock_monitor.py"
)
monitor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(monitor)


def test_evaluate_sample_detects_backward_and_offset_step():
    events = monitor.evaluate_sample((100, 1000), (200, 900), 50)
    assert {event["type"] for event in events} == {
        "realtime_backward",
        "offset_step",
    }


def test_evaluate_sample_accepts_matching_elapsed_clocks():
    assert monitor.evaluate_sample((100, 1000), (200, 1101), 5) == []


def test_monitor_validation_binds_live_process_session_and_coverage(
    tmp_path, monkeypatch
):
    path = tmp_path / "monitor.json"
    monkeypatch.setattr(monitor.time, "time_ns", lambda: 2_000_000_000)
    record = {
        "schema_version": 1,
        "kind": "robross_host_clock_monitor",
        "status": "running",
        "session_id": "session-a",
        "boot_id": "boot-a",
        "pid": os.getpid(),
        "updated_realtime_ns": 1_900_000_000,
        "coverage": {
            "clock_pair": "CLOCK_REALTIME/CLOCK_MONOTONIC",
            "sample_count": 2,
        },
    }
    path.write_text(json.dumps(record), encoding="utf-8")

    errors, loaded = monitor.validate_monitor(
        path, "session-a", max_update_age_s=1, expected_boot_id="boot-a"
    )
    assert errors == []
    assert loaded == record

    record["status"] = "failed"
    path.write_text(json.dumps(record), encoding="utf-8")
    errors, _loaded = monitor.validate_monitor(
        path, "session-a", max_update_age_s=1, expected_boot_id="boot-a"
    )
    assert "not healthy" in errors[0]
