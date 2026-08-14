#!/usr/bin/env python3
"""Extract joint-state seeds around a recorded painter command."""

import argparse
import json
import re
import sqlite3
from pathlib import Path

from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState


def bag_databases(bag_dir):
    databases = sorted(Path(bag_dir).glob("*.db3"))
    if not databases:
        raise RuntimeError(f"No .db3 file in {bag_dir}")
    return databases


def topic_id(connection, name):
    row = connection.execute("SELECT id FROM topics WHERE name = ?", (name,)).fetchone()
    if row is None:
        raise RuntimeError(f"Bag does not contain {name}")
    return row[0]


def command_storage_location(databases, command_index):
    pattern = re.compile(rf"^\[{command_index}/\d+\]")
    matches = []
    for split_ordinal, database in enumerate(databases):
        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        try:
            rosout_id = topic_id(connection, "/rosout")
            for storage_id, timestamp, data in connection.execute(
                "SELECT id, timestamp, data FROM messages WHERE topic_id = ? "
                "ORDER BY id", (rosout_id,)):
                message = deserialize_message(data, Log)
                if message.name == "painting_executor" and pattern.match(message.msg):
                    matches.append((split_ordinal, storage_id, timestamp))
        finally:
            connection.close()
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one log for command {command_index}, found {len(matches)}"
        )
    return matches[0]


def joint_samples(databases, target_location, sample_radius):
    target_split, target_id, _target_timestamp = target_location
    samples = []
    for split_ordinal, database in enumerate(databases):
        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        try:
            joint_state_id = topic_id(connection, "/joint_states")
            for storage_id, timestamp, data in connection.execute(
                "SELECT id, timestamp, data FROM messages WHERE topic_id = ? "
                "ORDER BY id", (joint_state_id,)):
                samples.append((split_ordinal, storage_id, timestamp, data))
        finally:
            connection.close()
    before = [row for row in samples
              if (row[0], row[1]) < (target_split, target_id)]
    after = [row for row in samples
             if (row[0], row[1]) > (target_split, target_id)]
    return before[-(sample_radius + 1):] + after[:sample_radius + 1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir")
    parser.add_argument("output_dir")
    parser.add_argument("--command-index", type=int, default=157)
    parser.add_argument("--sample-radius", type=int, default=2)
    args = parser.parse_args(argv)
    if args.command_index <= 0 or args.sample_radius < 0:
        parser.error("command index must be positive and sample radius non-negative")

    databases = bag_databases(args.bag_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    target = command_storage_location(databases, args.command_index)
    samples = joint_samples(databases, target, args.sample_radius)
    if not samples:
        raise RuntimeError("No joint-state samples found around the command")

    target_split, target_id, target_timestamp = target
    for ordinal, (split_ordinal, storage_id, timestamp, data) in enumerate(
            samples, start=1):
        message = deserialize_message(data, JointState)
        if len(message.name) != len(message.position) or not message.name:
            raise RuntimeError("Bag contains a malformed joint-state sample")
        payload = {
            "schema_version": 1,
            "joint_names": list(message.name),
            "joint_positions_rad": list(message.position),
            "exact_start_state": False,
            "source": str(databases[split_ordinal]),
            "command_index": args.command_index,
            "command_split_file_ordinal": target_split,
            "command_storage_id": target_id,
            "command_timestamp_ns": target_timestamp,
            "sample_split_file_ordinal": split_ordinal,
            "sample_storage_id": storage_id,
            "sample_timestamp_ns": timestamp,
            "sample_delta_ms": (timestamp - target_timestamp) / 1_000_000.0,
        }
        path = output_dir / f"command_{args.command_index}_seed_{ordinal:02d}.json"
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        print(f"{path}: delta={payload['sample_delta_ms']:+.3f} ms")


if __name__ == "__main__":
    main()
