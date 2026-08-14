import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / "scripts" / "host_time_preflight.py"
SPEC = importlib.util.spec_from_file_location("host_time_preflight", MODULE_PATH)
host_time = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(host_time)


def test_tracking_parser_and_thresholds():
    fields = host_time.parse_tracking(
        "Stratum : 3\nSystem time : 0.000012 seconds fast of NTP time\n"
        "Skew : 0.250 ppm\nLeap status : Normal\n"
    )

    assert host_time.tracking_errors(fields, 0.05, 5.0) == []
    assert host_time.tracking_errors(fields, 0.000001, 5.0)


def test_slew_only_config_follows_includes_and_rejects_makestep(tmp_path):
    included = tmp_path / "sources.conf"
    included.write_text("pool pool.ntp.org iburst\n", encoding="utf-8")
    root = tmp_path / "chrony.conf"
    root.write_text(f"include {included}\nrtcsync\n", encoding="utf-8")

    errors, hashes = host_time.check_slew_only_config(root)
    assert errors == []
    assert set(hashes) == {str(root), str(included)}

    included.write_text("pool pool.ntp.org\nmakestep 1.0 3\n", encoding="utf-8")
    errors, _hashes = host_time.check_slew_only_config(root)
    assert "makestep" in errors[0]


def test_performance_governor_requires_all_cpus(tmp_path):
    for cpu, governor in (("cpu0", "performance"), ("cpu1", "powersave")):
        path = tmp_path / cpu / "cpufreq"
        path.mkdir(parents=True)
        (path / "scaling_governor").write_text(governor, encoding="ascii")

    passed, details = host_time.performance_governor_check(tmp_path)
    assert passed is False
    assert "powersave" in details["governors"].values()


def test_atomic_receipt_validation_binds_session_boot_and_coverage(
    tmp_path, monkeypatch
):
    path = tmp_path / "receipt.json"
    monkeypatch.setattr(host_time.time, "time_ns", lambda: 1_005_000_000)
    receipt = {
        "schema_version": 1,
        "kind": "robross_host_time_preflight",
        "status": "pass",
        "session_id": "session-a",
        "boot_id": "boot-a",
        "created_realtime_ns": 1_000_000_000,
        "coverage": {name: True for name in host_time.REQUIRED_COVERAGE},
    }
    host_time.atomic_write_json(path, receipt)

    errors, loaded = host_time.validate_receipt(
        path, "session-a", max_age_s=1, expected_boot_id="boot-a"
    )
    assert errors == []
    assert loaded == receipt
    errors, _loaded = host_time.validate_receipt(
        path, "other", max_age_s=1, expected_boot_id="boot-a"
    )
    assert "session_id" in errors[0]

    errors, _loaded = host_time.validate_receipt(
        path, "session-a", max_age_s=1, expected_boot_id="boot-a",
        require_controller_fifo=True
    )
    assert "SCHED_FIFO" in errors[-1]


def test_running_controller_fifo_check_reads_thread_policy(tmp_path, monkeypatch):
    process = tmp_path / "123"
    (process / "task" / "456").mkdir(parents=True)
    (process / "cmdline").write_bytes(b"/overlay/aubo_ros2_control_node\0")
    monkeypatch.setattr(host_time.os, "sched_getscheduler", lambda tid: host_time.os.SCHED_FIFO)
    monkeypatch.setattr(
        host_time.os, "sched_getparam", lambda tid: SimpleNamespace(sched_priority=50)
    )
    passed, details = host_time.running_controller_fifo_check(proc_root=tmp_path)
    assert passed is True
    assert details["processes"][0]["threads"][0]["tid"] == 456


def test_preflight_writes_failure_receipt_when_chrony_unavailable(tmp_path):
    config = tmp_path / "chrony.conf"
    config.write_text("pool pool.ntp.org\n", encoding="utf-8")
    receipt = tmp_path / "receipt.json"
    args = SimpleNamespace(
        session_id="test",
        receipt=str(receipt),
        chrony_config=str(config),
        chrony_service="chrony",
        chronyc="chronyc",
        max_correction=0.05,
        max_skew=5.0,
        waitsync_tries=1,
        waitsync_interval=0.1,
        command_timeout=1.0,
    )

    def failing_runner(command, timeout):
        return SimpleNamespace(returncode=1, stdout="", stderr="unavailable")

    result = host_time.run_preflight(args, runner=failing_runner)
    assert result["status"] == "fail"
    assert json.loads(receipt.read_text(encoding="utf-8"))["status"] == "fail"
