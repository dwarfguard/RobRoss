#!/usr/bin/env python3
"""Publish healthy Aubo driver state for isolated fake-hardware tests only."""

import os

if os.environ.get("ROBROSS_FAKE_HEALTH") != "1":
    raise SystemExit("Refusing to publish without ROBROSS_FAKE_HEALTH=1")

try:
    domain_id = int(os.environ.get("ROS_DOMAIN_ID", "0"))
except ValueError as error:
    raise SystemExit("ROS_DOMAIN_ID must be a nonzero integer") from error
if domain_id == 0:
    raise SystemExit("Refusing to publish fake health on ROS_DOMAIN_ID 0")

import rclpy
from aubo_msgs.msg import DriverHealth
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class FakeHealthPublisher(Node):
    def __init__(self):
        super().__init__("fake_aubo_health_for_simulation_only")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            DriverHealth, "/aubo_driver/health", qos
        )
        self.sequence = 0
        self.timer = self.create_timer(0.05, self.publish_health)
        self.get_logger().warning(
            f"Publishing synthetic healthy driver state on isolated ROS domain {domain_id}"
        )

    def publish_health(self):
        message = DriverHealth()
        message.stamp = self.get_clock().now().to_msg()
        message.status_sequence = self.sequence
        message.activation_sequence = 1
        message.state = DriverHealth.READY
        message.fault_code = DriverHealth.FAULT_NONE
        message.fault_reason = "fake hardware simulation"
        message.writes_enabled = True
        message.source_sequence = self.sequence
        message.callback_count = self.sequence + 1
        message.valid_sample_count = self.sequence + 1
        message.source_age_sec = 0.001
        message.max_source_age_sec = 0.001
        message.callback_age_sec = 0.001
        message.requested_rate_hz = 200
        message.observed_rate_hz = 200.0
        message.interval_mean_ms = 5.0
        message.interval_p95_ms = 5.0
        message.interval_p99_ms = 5.0
        message.interval_max_ms = 5.0
        message.percentile_window_count = 1
        self.publisher.publish(message)
        self.sequence += 1


def main():
    rclpy.init()
    node = FakeHealthPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
