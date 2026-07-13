#!/usr/bin/env python3
"""Fail-closed DDS and RegScan continuity gate for the Office simulation."""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2


CRITICAL_TOPICS = (
    "/clock",
    "/imu/data",
    "/lidar/points",
    "/lidar/scan",
    "/registered_scan",
    "/state_estimation",
)


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class StreamGate(Node):
    def __init__(self) -> None:
        super().__init__("go2w_ros_stream_gate")
        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.clock_count = 0
        self.clock_regressions = 0
        self.clock_last_ns = None
        self.regscan_count = 0
        self.regscan_nonempty = 0
        self.regscan_regressions = 0
        self.regscan_last_ns = None
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.create_subscription(PointCloud2, "/registered_scan", self.on_regscan, qos)

    def on_clock(self, msg: Clock) -> None:
        current = stamp_ns(msg.clock)
        if self.clock_last_ns is not None and current < self.clock_last_ns:
            self.clock_regressions += 1
        self.clock_last_ns = current
        self.clock_count += 1

    def on_regscan(self, msg: PointCloud2) -> None:
        current = stamp_ns(msg.header.stamp)
        if self.regscan_last_ns is not None and current < self.regscan_last_ns:
            self.regscan_regressions += 1
        self.regscan_last_ns = current
        self.regscan_count += 1
        if msg.width * msg.height > 0 and len(msg.data) > 0:
            self.regscan_nonempty += 1

    def result(self, min_clock: int, min_regscan: int) -> dict:
        publishers = {topic: self.count_publishers(topic) for topic in CRITICAL_TOPICS}
        failures = []
        for topic, count in publishers.items():
            if count != 1:
                failures.append(f"{topic} publisher_count={count}")
        if self.clock_count < min_clock:
            failures.append(f"clock_count={self.clock_count}<{min_clock}")
        if self.clock_regressions:
            failures.append(f"clock_regressions={self.clock_regressions}")
        if self.regscan_count < min_regscan:
            failures.append(f"regscan_count={self.regscan_count}<{min_regscan}")
        if self.regscan_nonempty < min_regscan:
            failures.append(f"regscan_nonempty={self.regscan_nonempty}<{min_regscan}")
        if self.regscan_regressions:
            failures.append(f"regscan_regressions={self.regscan_regressions}")
        return {
            "ok": not failures,
            "publishers": publishers,
            "clock_count": self.clock_count,
            "clock_regressions": self.clock_regressions,
            "regscan_count": self.regscan_count,
            "regscan_nonempty": self.regscan_nonempty,
            "regscan_regressions": self.regscan_regressions,
            "failures": failures,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--min-clock", type=int, default=10)
    parser.add_argument("--min-regscan", type=int, default=2)
    args = parser.parse_args()
    rclpy.init()
    node = StreamGate()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        result = node.result(args.min_clock, args.min_regscan)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
