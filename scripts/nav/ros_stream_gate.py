#!/usr/bin/env python3
"""Fail-closed continuity gate for the Office simulation ROS streams."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import time
from typing import Mapping

try:
    import rclpy
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2
except ModuleNotFoundError as exc:  # Allow the pure gate logic to be unit tested off-robot.
    ROS_IMPORT_ERROR = exc
    rclpy = None
    CustomMsg = Odometry = Clock = Imu = PointCloud2 = object
    HistoryPolicy = QoSProfile = ReliabilityPolicy = None
    Node = object
else:
    ROS_IMPORT_ERROR = None


TOPICS = {
    "clock": "/clock",
    "imu": "/imu/data",
    "raw_cloud": "/lidar/points",
    "custom_scan": "/lidar/scan",
    "regscan": "/registered_scan",
    "state": "/state_estimation",
    "odom": "/odom_base_link",
}
CRITICAL_TOPICS = tuple(TOPICS.values())


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class StreamPolicy:
    min_count: int
    max_gap_s: float
    max_age_s: float | None = None
    require_nonempty: bool = False
    expected_frames: tuple[str, str] | None = None


@dataclass
class StreamStats:
    count: int = 0
    first_ns: int | None = None
    last_ns: int | None = None
    duplicate_stamps: int = 0
    regressions: int = 0
    max_gap_ns: int = 0
    nonempty_count: int = 0
    valid_frame_count: int = 0
    last_wall_s: float | None = None

    def observe(
        self,
        current_ns: int,
        wall_s: float,
        *,
        nonempty: bool = True,
        valid_frames: bool = True,
    ) -> None:
        if self.first_ns is None:
            self.first_ns = current_ns
        if self.last_ns is not None:
            delta_ns = current_ns - self.last_ns
            if delta_ns < 0:
                self.regressions += 1
            elif delta_ns == 0:
                self.duplicate_stamps += 1
            else:
                self.max_gap_ns = max(self.max_gap_ns, delta_ns)
        self.last_ns = current_ns
        self.last_wall_s = wall_s
        self.count += 1
        self.nonempty_count += int(nonempty)
        self.valid_frame_count += int(valid_frames)

    def as_dict(self, clock_last_ns: int | None) -> dict:
        sim_age_s = None
        if clock_last_ns is not None and self.last_ns is not None:
            sim_age_s = (clock_last_ns - self.last_ns) / 1e9
        return {
            "count": self.count,
            "first_ns": self.first_ns,
            "last_ns": self.last_ns,
            "duplicate_stamps": self.duplicate_stamps,
            "regressions": self.regressions,
            "max_gap_s": self.max_gap_ns / 1e9,
            "sim_age_s": sim_age_s,
            "nonempty_count": self.nonempty_count,
            "valid_frame_count": self.valid_frame_count,
        }


def evaluate_streams(
    stats: Mapping[str, StreamStats],
    policies: Mapping[str, StreamPolicy],
    publishers: Mapping[str, int],
    *,
    now_wall_s: float,
    max_future_skew_s: float,
    max_clock_wall_age_s: float,
) -> dict:
    """Evaluate a snapshot without using wall time for non-clock stream rates."""
    failures: list[str] = []
    for topic in CRITICAL_TOPICS:
        count = publishers.get(topic, 0)
        if count != 1:
            failures.append(f"{topic} publisher_count={count}")

    clock = stats["clock"]
    if clock.last_wall_s is None:
        failures.append("clock_wall_stamp_missing")
    else:
        clock_wall_age_s = now_wall_s - clock.last_wall_s
        if clock_wall_age_s < 0 or clock_wall_age_s > max_clock_wall_age_s:
            failures.append(
                f"clock_wall_age_s={clock_wall_age_s:.6f}>"
                f"{max_clock_wall_age_s:.6f}"
            )

    for name, policy in policies.items():
        stream = stats[name]
        if stream.count < policy.min_count:
            failures.append(f"{name}_count={stream.count}<{policy.min_count}")
        if stream.duplicate_stamps:
            failures.append(f"{name}_duplicate_stamps={stream.duplicate_stamps}")
        if stream.regressions:
            failures.append(f"{name}_regressions={stream.regressions}")
        if (
            stream.count >= 2
            and stream.first_ns is not None
            and stream.last_ns is not None
            and stream.last_ns <= stream.first_ns
        ):
            if name == "odom":
                failures.append("odom_stamp_not_advancing")
            else:
                failures.append(f"{name}_stamp_not_advancing")
        max_gap_s = stream.max_gap_ns / 1e9
        if stream.count >= 2 and max_gap_s > policy.max_gap_s:
            failures.append(
                f"{name}_max_gap_s={max_gap_s:.6f}>{policy.max_gap_s:.6f}"
            )
        if policy.require_nonempty and stream.nonempty_count != stream.count:
            failures.append(
                f"{name}_nonempty={stream.nonempty_count}!={stream.count}"
            )
        if policy.expected_frames and stream.valid_frame_count != stream.count:
            failures.append(
                f"{name}_valid_frames={stream.valid_frame_count}!={stream.count}"
            )

        if name == "clock" or clock.last_ns is None or stream.last_ns is None:
            continue
        sim_age_s = (clock.last_ns - stream.last_ns) / 1e9
        if sim_age_s < -max_future_skew_s:
            failures.append(
                f"{name}_future_skew_s={-sim_age_s:.6f}>"
                f"{max_future_skew_s:.6f}"
            )
        elif policy.max_age_s is not None and sim_age_s > policy.max_age_s:
            failures.append(
                f"{name}_sim_age_s={sim_age_s:.6f}>{policy.max_age_s:.6f}"
            )

    stream_results = {
        name: stream.as_dict(clock.last_ns) for name, stream in stats.items()
    }
    return {
        "ok": not failures,
        "publishers": dict(publishers),
        "streams": stream_results,
        "failures": failures,
    }


class StreamGate(Node):
    def __init__(self) -> None:
        super().__init__("go2w_ros_stream_gate")
        qos = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.stats = {name: StreamStats() for name in TOPICS}
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos)
        self.create_subscription(
            PointCloud2, "/lidar/points", self.on_raw_cloud, qos
        )
        self.create_subscription(
            CustomMsg, "/lidar/scan", self.on_custom_scan, qos
        )
        self.create_subscription(
            PointCloud2, "/registered_scan", self.on_regscan, qos
        )
        self.create_subscription(
            Odometry, "/state_estimation", self.on_state, qos
        )
        self.create_subscription(Odometry, "/odom_base_link", self.on_odom, qos)

    def _observe(
        self,
        name: str,
        current_ns: int,
        *,
        nonempty: bool = True,
        valid_frames: bool = True,
    ) -> None:
        self.stats[name].observe(
            current_ns,
            time.monotonic(),
            nonempty=nonempty,
            valid_frames=valid_frames,
        )

    def on_clock(self, msg: Clock) -> None:
        self._observe("clock", stamp_ns(msg.clock))

    def on_imu(self, msg: Imu) -> None:
        self._observe("imu", stamp_ns(msg.header.stamp))

    def on_raw_cloud(self, msg: PointCloud2) -> None:
        self._observe(
            "raw_cloud",
            stamp_ns(msg.header.stamp),
            nonempty=msg.width * msg.height > 0 and len(msg.data) > 0,
        )

    def on_custom_scan(self, msg: CustomMsg) -> None:
        self._observe(
            "custom_scan",
            stamp_ns(msg.header.stamp),
            nonempty=msg.point_num > 0 and len(msg.points) > 0,
        )

    def on_regscan(self, msg: PointCloud2) -> None:
        self._observe(
            "regscan",
            stamp_ns(msg.header.stamp),
            nonempty=msg.width * msg.height > 0 and len(msg.data) > 0,
        )

    def on_state(self, msg: Odometry) -> None:
        self._observe(
            "state",
            stamp_ns(msg.header.stamp),
            valid_frames=(
                msg.header.frame_id == "map" and msg.child_frame_id == "sensor"
            ),
        )

    def on_odom(self, msg: Odometry) -> None:
        self._observe(
            "odom",
            stamp_ns(msg.header.stamp),
            valid_frames=(
                msg.header.frame_id == "map" and msg.child_frame_id == "base_link"
            ),
        )

    def result(
        self,
        policies: Mapping[str, StreamPolicy],
        *,
        max_future_skew_s: float,
        max_clock_wall_age_s: float,
    ) -> dict:
        publishers = {topic: self.count_publishers(topic) for topic in CRITICAL_TOPICS}
        result = evaluate_streams(
            self.stats,
            policies,
            publishers,
            now_wall_s=time.monotonic(),
            max_future_skew_s=max_future_skew_s,
            max_clock_wall_age_s=max_clock_wall_age_s,
        )
        # Preserve common log fields for existing diagnostics while exposing all
        # stream metrics under ``streams``.
        result.update(
            clock_count=self.stats["clock"].count,
            regscan_count=self.stats["regscan"].count,
            regscan_nonempty=self.stats["regscan"].nonempty_count,
            odom_count=self.stats["odom"].count,
            odom_valid_frames=self.stats["odom"].valid_frame_count,
        )
        return result


def at_least_two(value: str) -> int:
    parsed = int(value)
    if parsed < 2:
        raise argparse.ArgumentTypeError("continuity requires at least two frames")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=positive_float, default=12.0)
    parser.add_argument("--min-clock", type=at_least_two, default=10)
    parser.add_argument("--min-imu", type=at_least_two, default=2)
    parser.add_argument("--min-raw", type=at_least_two, default=2)
    parser.add_argument("--min-custom", type=at_least_two, default=2)
    parser.add_argument("--min-regscan", type=at_least_two, default=2)
    parser.add_argument("--min-state", type=at_least_two, default=2)
    parser.add_argument("--min-odom", type=at_least_two, default=2)
    parser.add_argument("--max-clock-gap", type=positive_float, default=0.25)
    parser.add_argument("--max-imu-gap", type=positive_float, default=0.25)
    parser.add_argument("--max-raw-gap", type=positive_float, default=0.25)
    parser.add_argument("--max-custom-gap", type=positive_float, default=1.0)
    parser.add_argument("--max-regscan-gap", type=positive_float, default=1.5)
    parser.add_argument("--max-state-gap", type=positive_float, default=0.5)
    parser.add_argument("--max-odom-gap", type=positive_float, default=0.5)
    parser.add_argument("--max-imu-age", type=positive_float, default=0.25)
    parser.add_argument("--max-raw-age", type=positive_float, default=0.25)
    parser.add_argument("--max-custom-age", type=positive_float, default=1.0)
    parser.add_argument("--max-regscan-age", type=positive_float, default=1.5)
    parser.add_argument("--max-state-age", type=positive_float, default=0.5)
    parser.add_argument("--max-odom-age", type=positive_float, default=0.5)
    parser.add_argument("--max-future-skew", type=positive_float, default=0.1)
    parser.add_argument("--max-clock-wall-age", type=positive_float, default=2.0)
    return parser.parse_args()


def policies_from_args(args: argparse.Namespace) -> dict[str, StreamPolicy]:
    return {
        "clock": StreamPolicy(args.min_clock, args.max_clock_gap),
        "imu": StreamPolicy(args.min_imu, args.max_imu_gap, args.max_imu_age),
        "raw_cloud": StreamPolicy(
            args.min_raw,
            args.max_raw_gap,
            args.max_raw_age,
            require_nonempty=True,
        ),
        "custom_scan": StreamPolicy(
            args.min_custom,
            args.max_custom_gap,
            args.max_custom_age,
            require_nonempty=True,
        ),
        "regscan": StreamPolicy(
            args.min_regscan,
            args.max_regscan_gap,
            args.max_regscan_age,
            require_nonempty=True,
        ),
        "state": StreamPolicy(
            args.min_state,
            args.max_state_gap,
            args.max_state_age,
            expected_frames=("map", "sensor"),
        ),
        "odom": StreamPolicy(
            args.min_odom,
            args.max_odom_gap,
            args.max_odom_age,
            expected_frames=("map", "base_link"),
        ),
    }


def main() -> int:
    if rclpy is None:
        raise RuntimeError(f"ROS 2 Python dependencies are unavailable: {ROS_IMPORT_ERROR}")
    args = parse_args()
    rclpy.init()
    node = StreamGate()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        result = node.result(
            policies_from_args(args),
            max_future_skew_s=args.max_future_skew,
            max_clock_wall_age_s=args.max_clock_wall_age,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
