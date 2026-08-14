#!/usr/bin/env python3
"""Record realtime-vs-monotonic stability for a RobRoss hardware session."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import time

from host_time_preflight import atomic_write_json, boot_id, utc_now


SCHEMA_VERSION = 1


def sample_clocks():
    monotonic_ns = time.monotonic_ns()
    realtime_ns = time.time_ns()
    return monotonic_ns, realtime_ns


def evaluate_sample(previous, current, max_offset_step_ns):
    if previous is None:
        return []
    previous_mono, previous_real = previous
    monotonic_ns, realtime_ns = current
    errors = []
    if realtime_ns < previous_real:
        errors.append(
            {
                "type": "realtime_backward",
                "magnitude_ns": previous_real - realtime_ns,
            }
        )
    elapsed_mono = monotonic_ns - previous_mono
    elapsed_real = realtime_ns - previous_real
    offset_step = elapsed_real - elapsed_mono
    if abs(offset_step) > max_offset_step_ns:
        errors.append({"type": "offset_step", "magnitude_ns": offset_step})
    return errors


def initial_record(args, first):
    monotonic_ns, realtime_ns = first
    return {
        "schema_version": SCHEMA_VERSION,
        "kind": "robross_host_clock_monitor",
        "status": "running",
        "session_id": args.session_id,
        "host": socket.gethostname(),
        "boot_id": boot_id(),
        "pid": os.getpid(),
        "started_utc": utc_now(),
        "started_realtime_ns": realtime_ns,
        "updated_realtime_ns": realtime_ns,
        "coverage": {
            "clock_pair": "CLOCK_REALTIME/CLOCK_MONOTONIC",
            "sample_interval_s": args.interval,
            "max_offset_step_ns": args.max_offset_step_ns,
            "first_monotonic_ns": monotonic_ns,
            "last_monotonic_ns": monotonic_ns,
            "first_realtime_ns": realtime_ns,
            "last_realtime_ns": realtime_ns,
            "sample_count": 1,
            "covered_monotonic_ns": 0,
        },
        "max_abs_offset_step_ns": 0,
        "backward_event_count": 0,
        "offset_step_event_count": 0,
        "events": [],
    }


def monitor(args, sample_fn=sample_clocks, sleep_fn=time.sleep):
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    first = sample_fn()
    previous = first
    record = initial_record(args, first)
    atomic_write_json(args.output, record)
    while not stopping and not (args.stop_file and Path(args.stop_file).exists()):
        sleep_fn(args.interval)
        current = sample_fn()
        elapsed_mono = current[0] - previous[0]
        elapsed_real = current[1] - previous[1]
        offset_step = elapsed_real - elapsed_mono
        record["max_abs_offset_step_ns"] = max(
            record["max_abs_offset_step_ns"], abs(offset_step)
        )
        for event in evaluate_sample(previous, current, args.max_offset_step_ns):
            event.update(
                {
                    "sample": record["coverage"]["sample_count"] + 1,
                    "monotonic_ns": current[0],
                    "realtime_ns": current[1],
                }
            )
            if event["type"] == "realtime_backward":
                record["backward_event_count"] += 1
            else:
                record["offset_step_event_count"] += 1
            if len(record["events"]) < args.max_events:
                record["events"].append(event)
        coverage = record["coverage"]
        coverage["sample_count"] += 1
        coverage["last_monotonic_ns"] = current[0]
        coverage["last_realtime_ns"] = current[1]
        coverage["covered_monotonic_ns"] = current[0] - first[0]
        record["updated_realtime_ns"] = current[1]
        record["status"] = (
            "failed"
            if record["backward_event_count"] or record["offset_step_event_count"]
            else "running"
        )
        atomic_write_json(args.output, record)
        previous = current
        if args.fail_fast and record["status"] == "failed":
            break
        if args.samples and coverage["sample_count"] >= args.samples:
            break
    record["ended_utc"] = utc_now()
    record["status"] = (
        "failed"
        if record["backward_event_count"] or record["offset_step_event_count"]
        else "complete"
    )
    atomic_write_json(args.output, record)
    return 1 if record["status"] == "failed" else 0


def validate_monitor(path, session_id, max_update_age_s=5.0, expected_boot_id=None):
    try:
        record = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return [f"cannot read clock monitor: {error}"], None
    errors = []
    if record.get("schema_version") != SCHEMA_VERSION or record.get("kind") != "robross_host_clock_monitor":
        errors.append("invalid clock monitor schema/kind")
    if record.get("session_id") != session_id:
        errors.append("clock monitor session_id does not match")
    if record.get("boot_id") != (expected_boot_id or boot_id()):
        errors.append("clock monitor was not started during this boot")
    if record.get("status") != "running":
        errors.append(f"clock monitor is not healthy/running: {record.get('status')!r}")
    age_s = (time.time_ns() - record.get("updated_realtime_ns", 0)) / 1e9
    if age_s < -1 or age_s > max_update_age_s:
        errors.append(f"clock monitor update age {age_s:.1f}s is stale")
    coverage = record.get("coverage", {})
    if coverage.get("sample_count", 0) < 1 or coverage.get("clock_pair") != "CLOCK_REALTIME/CLOCK_MONOTONIC":
        errors.append("clock monitor lacks required clock-pair coverage")
    pid = record.get("pid")
    if not isinstance(pid, int) or not Path(f"/proc/{pid}").exists():
        errors.append("clock monitor process is not alive")
    return errors, record


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--stop-file", default="")
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--max-offset-step-ns", type=int, default=5_000_000)
    parser.add_argument("--max-events", type=int, default=100)
    parser.add_argument("--fail-fast", action="store_true")
    parser.add_argument("--samples", type=int, default=0, help="Testing/finite-run sample count")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--max-update-age", type=float, default=5.0)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.validate_only:
        errors, _record = validate_monitor(
            args.output, args.session_id, args.max_update_age
        )
        if errors:
            print("Host clock monitor rejected: " + "; ".join(errors))
            return 1
        print(f"Host clock monitor accepted: {args.output}")
        return 0
    if args.interval <= 0 or args.max_offset_step_ns <= 0 or args.max_events < 1:
        raise SystemExit("interval, max-offset-step-ns, and max-events must be positive")
    return monitor(args)


if __name__ == "__main__":
    raise SystemExit(main())
