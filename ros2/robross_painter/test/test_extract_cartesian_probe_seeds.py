import importlib.util
import json
import sqlite3
from pathlib import Path

from rcl_interfaces.msg import Log
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_ROOT / "scripts" / "extract_cartesian_probe_seeds.py"
SPEC = importlib.util.spec_from_file_location("extract_probe_seeds", SCRIPT)
extract_probe_seeds = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(extract_probe_seeds)


def create_bag(path):
    connection = sqlite3.connect(path)
    connection.executescript(
        """
        CREATE TABLE topics(
          id INTEGER PRIMARY KEY,
          name TEXT NOT NULL,
          type TEXT NOT NULL,
          serialization_format TEXT NOT NULL,
          offered_qos_profiles TEXT NOT NULL);
        CREATE TABLE messages(
          id INTEGER PRIMARY KEY,
          topic_id INTEGER NOT NULL,
          timestamp INTEGER NOT NULL,
          data BLOB NOT NULL);
        """
    )
    connection.execute(
        "INSERT INTO topics VALUES(1, '/rosout', 'rcl_interfaces/msg/Log', 'cdr', '')"
    )
    connection.execute(
        "INSERT INTO topics VALUES(2, '/joint_states', 'sensor_msgs/msg/JointState', 'cdr', '')"
    )
    entries = [(2, 1_010, 1.0), (1, 1_000, None), (2, 990, 1.1)]
    for topic_id, timestamp, position in entries:
        if topic_id == 1:
            message = Log()
            message.name = "painting_executor"
            message.msg = "[157/1466] paint_path (line_art_line_39)"
        else:
            message = JointState()
            message.name = ["joint_a"]
            message.position = [position]
        connection.execute(
            "INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
            (topic_id, timestamp, serialize_message(message)),
        )
    connection.commit()
    connection.close()


def test_extracts_bracketing_approximate_seeds(tmp_path):
    bag_dir = tmp_path / "bag"
    bag_dir.mkdir()
    create_bag(str(bag_dir / "bag.db3"))
    output_dir = tmp_path / "seeds"

    extract_probe_seeds.main(
        [str(bag_dir), str(output_dir), "--command-index", "157", "--sample-radius", "0"]
    )

    seeds = sorted(output_dir.glob("*.json"))
    assert len(seeds) == 2
    payloads = [json.loads(path.read_text(encoding="utf-8")) for path in seeds]
    assert [payload["sample_delta_ms"] for payload in payloads] == [0.00001, -0.00001]
    assert [payload["sample_storage_id"] for payload in payloads] == [1, 3]
    assert all(payload["command_storage_id"] == 2 for payload in payloads)
    assert all(payload["exact_start_state"] is False for payload in payloads)


def test_duplicate_timestamps_bracket_by_storage_id(tmp_path):
    bag_dir = tmp_path / "bag"
    bag_dir.mkdir()
    create_bag(str(bag_dir / "bag.db3"))
    connection = sqlite3.connect(bag_dir / "bag.db3")
    connection.execute("UPDATE messages SET timestamp = 1000")
    connection.commit()
    connection.close()
    databases = extract_probe_seeds.bag_databases(bag_dir)
    target = extract_probe_seeds.command_storage_location(databases, 157)
    samples = extract_probe_seeds.joint_samples(databases, target, 0)
    assert [sample[1] for sample in samples] == [1, 3]
