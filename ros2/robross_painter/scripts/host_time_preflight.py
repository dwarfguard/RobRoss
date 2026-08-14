#!/usr/bin/env python3
"""Fail-closed Chrony/host-clock gate for physical robot sessions."""

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import socket
import subprocess
import sys
import tempfile
import time


SCHEMA_VERSION = 1
REQUIRED_COVERAGE = (
    "chrony_tracking",
    "chrony_waitsync",
    "slew_only_config",
    "single_synchronizer",
    "realtime_priority",
    "performance_governor",
    "clock_sample",
)
COMPETING_SERVICES = ("systemd-timesyncd", "ntp", "ntpd", "openntpd")


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def boot_id(path="/proc/sys/kernel/random/boot_id"):
    try:
        return Path(path).read_text(encoding="ascii").strip()
    except OSError:
        return "unknown"


def atomic_write_json(path, payload):
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(payload, output, indent=2, sort_keys=True)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, destination)
        directory_fd = os.open(destination.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def parse_tracking(text):
    fields = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        fields[key.strip().lower().replace(" ", "_")] = value.strip()
    return fields


def seconds_value(value):
    match = re.search(r"[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", value or "")
    if not match:
        raise ValueError(f"no numeric value in {value!r}")
    return float(match.group(0))


def tracking_errors(fields, max_correction_s, max_skew_ppm):
    errors = []
    if fields.get("leap_status", "").lower() != "normal":
        errors.append("Chrony leap status is not Normal")
    try:
        if int(fields.get("stratum", "0")) <= 0:
            errors.append("Chrony stratum is not synchronized")
    except ValueError:
        errors.append("Chrony stratum is invalid")
    try:
        correction = abs(seconds_value(fields.get("system_time")))
        if correction > max_correction_s:
            errors.append(
                f"Chrony system-time correction {correction:.6f}s exceeds "
                f"{max_correction_s:.6f}s"
            )
    except ValueError as error:
        errors.append(f"Chrony system-time correction is invalid: {error}")
    try:
        skew = abs(seconds_value(fields.get("skew")))
        if skew > max_skew_ppm:
            errors.append(
                f"Chrony skew {skew:.3f}ppm exceeds {max_skew_ppm:.3f}ppm"
            )
    except ValueError as error:
        errors.append(f"Chrony skew is invalid: {error}")
    return errors


def read_chrony_config(root_path):
    """Return recursively included config files and active directives."""
    files = []
    directives = []
    pending = [Path(root_path).expanduser().resolve()]
    visited = set()
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        visited.add(path)
        text = path.read_text(encoding="utf-8")
        files.append(path)
        for line_number, raw_line in enumerate(text.splitlines(), 1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            directive = parts[0].lower()
            directives.append((path, line_number, directive, parts[1:]))
            if directive == "include":
                for pattern in parts[1:]:
                    include = Path(pattern)
                    if not include.is_absolute():
                        include = path.parent / include
                    pending.extend(sorted(include.parent.glob(include.name)))
            elif directive == "confdir":
                for directory in parts[1:]:
                    include_dir = Path(directory)
                    if not include_dir.is_absolute():
                        include_dir = path.parent / include_dir
                    if include_dir.is_dir():
                        pending.extend(sorted(include_dir.glob("*.conf")))
    return sorted(files), directives


def check_slew_only_config(root_path):
    files, directives = read_chrony_config(root_path)
    forbidden = []
    sources = 0
    for path, line_number, directive, _arguments in directives:
        if directive in ("makestep", "initstepslew"):
            forbidden.append(f"{path}:{line_number}: {directive}")
        if directive in ("server", "pool", "peer"):
            sources += 1
    errors = []
    if forbidden:
        errors.append("step-capable makestep directive(s): " + ", ".join(forbidden))
    if sources == 0:
        errors.append("no Chrony server/pool/peer source configured")
    hashes = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files
    }
    return errors, hashes


def run_command(command, timeout=30):
    return subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def service_active(service, runner=run_command):
    result = runner(["systemctl", "is-active", "--quiet", service], timeout=5)
    return result.returncode == 0


def clock_sample():
    mono_before = time.monotonic_ns()
    realtime = time.time_ns()
    mono_after = time.monotonic_ns()
    return {
        "monotonic_before_ns": mono_before,
        "realtime_ns": realtime,
        "monotonic_after_ns": mono_after,
        "realtime_minus_monotonic_ns": realtime - ((mono_before + mono_after) // 2),
    }


def realtime_priority_check(required_priority=50):
    soft, hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
    passed = soft == resource.RLIM_INFINITY or soft >= required_priority
    scheduler = os.sched_getscheduler(0)
    scheduler_names = {
        os.SCHED_OTHER: "SCHED_OTHER",
        os.SCHED_FIFO: "SCHED_FIFO",
        os.SCHED_RR: "SCHED_RR",
    }
    return passed, {
        "soft_limit": soft,
        "hard_limit": hard,
        "required": required_priority,
        "preflight_scheduler": scheduler_names.get(scheduler, str(scheduler)),
        "preflight_priority": os.sched_getparam(0).sched_priority,
    }


def running_controller_fifo_check(
    process_name="aubo_ros2_control_node", required_priority=50,
    proc_root="/proc"
):
    matches = []
    for process_dir in Path(proc_root).glob("[0-9]*"):
        try:
            command = (process_dir / "cmdline").read_bytes().replace(b"\0", b" ").decode()
        except (OSError, UnicodeDecodeError):
            continue
        if process_name not in command:
            continue
        process = {"pid": int(process_dir.name), "threads": []}
        for task_dir in (process_dir / "task").glob("[0-9]*"):
            tid = int(task_dir.name)
            try:
                policy = os.sched_getscheduler(tid)
                priority = os.sched_getparam(tid).sched_priority
            except (OSError, PermissionError):
                continue
            process["threads"].append({
                "tid": tid,
                "policy": policy,
                "priority": priority,
            })
        matches.append(process)
    passed = any(
        thread["policy"] == os.SCHED_FIFO and
        thread["priority"] >= required_priority
        for process in matches for thread in process["threads"]
    )
    return passed, {
        "process_name": process_name,
        "required_priority": required_priority,
        "processes": matches,
    }


def performance_governor_check(sysfs_root="/sys/devices/system/cpu"):
    paths = sorted(Path(sysfs_root).glob("cpu[0-9]*/cpufreq/scaling_governor"))
    if not paths:
        return False, {"governors": {}, "error": "CPU scaling governors unavailable"}
    governors = {}
    for path in paths:
        try:
            governors[str(path)] = path.read_text(encoding="ascii").strip()
        except OSError as error:
            return False, {"governors": governors, "error": str(error)}
    passed = all(value == "performance" for value in governors.values())
    return passed, {"governors": governors}


def run_preflight(args, runner=run_command):
    errors = []
    checks = {}
    coverage = {name: False for name in REQUIRED_COVERAGE}

    try:
        config_errors, config_hashes = check_slew_only_config(args.chrony_config)
        coverage["slew_only_config"] = not config_errors
        checks["slew_only_config"] = {
            "passed": not config_errors,
            "root": str(Path(args.chrony_config).expanduser().resolve()),
            "files_sha256": config_hashes,
            "errors": config_errors,
        }
        errors.extend(config_errors)
    except OSError as error:
        message = f"cannot inspect Chrony config: {error}"
        checks["slew_only_config"] = {"passed": False, "errors": [message]}
        errors.append(message)

    active_competitors = []
    try:
        chrony_active = service_active(args.chrony_service, runner)
        for service in COMPETING_SERVICES:
            if service_active(service, runner):
                active_competitors.append(service)
        single = chrony_active and not active_competitors
        coverage["single_synchronizer"] = single
        checks["single_synchronizer"] = {
            "passed": single,
            "chrony_service": args.chrony_service,
            "chrony_active": chrony_active,
            "active_competitors": active_competitors,
        }
        if not chrony_active:
            errors.append(f"{args.chrony_service} is not active")
        if active_competitors:
            errors.append("competing time services active: " + ", ".join(active_competitors))
    except (OSError, subprocess.SubprocessError) as error:
        message = f"cannot determine active time synchronizers: {error}"
        checks["single_synchronizer"] = {"passed": False, "errors": [message]}
        errors.append(message)

    realtime_passed, realtime_details = realtime_priority_check()
    coverage["realtime_priority"] = realtime_passed
    checks["realtime_priority"] = {"passed": realtime_passed, **realtime_details}
    if not realtime_passed:
        errors.append("RLIMIT_RTPRIO does not permit required FIFO priority 50")

    controller_fifo_passed, controller_fifo_details = \
        running_controller_fifo_check()
    checks["controller_fifo"] = {
        "passed": controller_fifo_passed,
        **controller_fifo_details,
    }
    if getattr(args, "require_controller_fifo", False) and not controller_fifo_passed:
        errors.append("running aubo_ros2_control_node is not SCHED_FIFO/50")

    governor_passed, governor_details = performance_governor_check()
    coverage["performance_governor"] = governor_passed
    checks["performance_governor"] = {"passed": governor_passed, **governor_details}
    if not governor_passed:
        errors.append("not all CPU scaling governors are set to performance")

    tracking_fields = {}
    try:
        tracking = runner([args.chronyc, "tracking"], timeout=args.command_timeout)
        tracking_fields = parse_tracking(tracking.stdout)
        tracking_failure = []
        if tracking.returncode != 0:
            tracking_failure.append(tracking.stderr.strip() or "chronyc tracking failed")
        else:
            tracking_failure.extend(
                tracking_errors(
                    tracking_fields, args.max_correction, args.max_skew
                )
            )
        coverage["chrony_tracking"] = not tracking_failure
        checks["chrony_tracking"] = {
            "passed": not tracking_failure,
            "fields": tracking_fields,
            "errors": tracking_failure,
        }
        errors.extend(tracking_failure)
    except (OSError, subprocess.SubprocessError) as error:
        message = f"cannot run chronyc tracking: {error}"
        checks["chrony_tracking"] = {"passed": False, "errors": [message]}
        errors.append(message)

    try:
        waitsync = runner(
            [
                args.chronyc,
                "waitsync",
                str(args.waitsync_tries),
                str(args.max_correction),
                str(args.max_skew),
                str(args.waitsync_interval),
            ],
            timeout=args.command_timeout,
        )
        waitsync_passed = waitsync.returncode == 0
        coverage["chrony_waitsync"] = waitsync_passed
        checks["chrony_waitsync"] = {
            "passed": waitsync_passed,
            "stdout": waitsync.stdout.strip(),
            "stderr": waitsync.stderr.strip(),
        }
        if not waitsync_passed:
            errors.append(waitsync.stderr.strip() or "chronyc waitsync failed")
    except (OSError, subprocess.SubprocessError) as error:
        message = f"cannot run chronyc waitsync: {error}"
        checks["chrony_waitsync"] = {"passed": False, "errors": [message]}
        errors.append(message)

    sample = clock_sample()
    coverage["clock_sample"] = True
    checks["clock_sample"] = {"passed": True, **sample}

    receipt = {
        "schema_version": SCHEMA_VERSION,
        "kind": "robross_host_time_preflight",
        "status": "pass" if not errors and all(coverage.values()) else "fail",
        "session_id": args.session_id,
        "created_utc": utc_now(),
        "created_realtime_ns": time.time_ns(),
        "host": socket.gethostname(),
        "boot_id": boot_id(),
        "pid": os.getpid(),
        "coverage": coverage,
        "checks": checks,
        "errors": errors,
    }
    atomic_write_json(args.receipt, receipt)
    return receipt


def validate_receipt(path, session_id, max_age_s=600, expected_boot_id=None,
                     require_controller_fifo=False):
    try:
        receipt = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return [f"cannot read preflight receipt: {error}"], None
    errors = []
    if receipt.get("schema_version") != SCHEMA_VERSION:
        errors.append("unsupported preflight receipt schema")
    if receipt.get("kind") != "robross_host_time_preflight":
        errors.append("not a RobRoss host-time preflight receipt")
    if receipt.get("status") != "pass":
        errors.append("preflight receipt does not have pass status")
    if receipt.get("session_id") != session_id:
        errors.append("preflight receipt session_id does not match")
    current_boot = expected_boot_id or boot_id()
    if receipt.get("boot_id") != current_boot:
        errors.append("preflight receipt was not created during this boot")
    age_s = (time.time_ns() - receipt.get("created_realtime_ns", 0)) / 1e9
    if age_s < -1 or age_s > max_age_s:
        errors.append(f"preflight receipt age {age_s:.1f}s is outside the allowed window")
    coverage = receipt.get("coverage", {})
    missing = [name for name in REQUIRED_COVERAGE if coverage.get(name) is not True]
    if missing:
        errors.append("preflight receipt lacks coverage: " + ", ".join(missing))
    if require_controller_fifo and \
            receipt.get("checks", {}).get("controller_fifo", {}).get("passed") is not True:
        errors.append("preflight receipt does not prove controller SCHED_FIFO/50")
    return errors, receipt


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--chrony-config", default="/etc/chrony/chrony.conf")
    parser.add_argument("--chrony-service", default="chrony")
    parser.add_argument("--chronyc", default="chronyc")
    parser.add_argument("--max-correction", type=float, default=0.050)
    parser.add_argument("--max-skew", type=float, default=5.0)
    parser.add_argument("--waitsync-tries", type=int, default=60)
    parser.add_argument("--waitsync-interval", type=float, default=1.0)
    parser.add_argument("--command-timeout", type=float, default=75.0)
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--max-receipt-age", type=float, default=600.0)
    parser.add_argument("--require-controller-fifo", action="store_true")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.validate_only:
        errors, _receipt = validate_receipt(
            args.receipt, args.session_id, args.max_receipt_age,
            require_controller_fifo=args.require_controller_fifo
        )
        if errors:
            print("Host time preflight receipt rejected: " + "; ".join(errors), file=sys.stderr)
            return 1
        print(f"Host time preflight receipt accepted: {args.receipt}")
        return 0
    receipt = run_preflight(args)
    if receipt["status"] != "pass":
        print("Host time preflight failed: " + "; ".join(receipt["errors"]), file=sys.stderr)
        return 1
    print(f"Host time preflight passed: {args.receipt}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
