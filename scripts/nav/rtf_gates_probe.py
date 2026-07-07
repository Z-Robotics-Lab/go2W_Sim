#!/usr/bin/env python3
"""低 RTF 渲染矩阵闸门探针（RTF + 雷达率/点率 + SLAM z 峰峰）。

一个 rclpy 节点并发采三闸，写 JSON 到 stdout 末行（前缀 RESULT_JSON=）。
- RTF: /clock 首末 sim 秒差 / 墙钟差（wall = time.time()）。
- 雷达闸: /lidar/scan (livox CustomMsg) 消息数->hz，sum(point_num)->点率(pts/s)。
- SLAM z 闸: /state_estimation (Odometry) z 峰峰值。

用法: python3 rtf_gates_probe.py <window_s>
纯只读，不发任何指令、不重启、不杀进程。
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

# livox CustomMsg（可能不在默认 import 路径，惰性容错）
try:
    from livox_ros_driver2.msg import CustomMsg
    HAVE_LIVOX = True
except Exception:
    CustomMsg = None
    HAVE_LIVOX = False


class GatesProbe(Node):
    def __init__(self, window_s: float):
        super().__init__("rtf_gates_probe")
        self.window_s = window_s
        self.wall0 = None
        self.wall1 = None
        self.sim0 = None
        self.sim1 = None
        # 雷达
        self.scan_msgs = 0
        self.scan_points = 0
        self.scan_first_wall = None
        self.scan_last_wall = None
        # SLAM z
        self.z_min = None
        self.z_max = None
        self.z_samples = 0

        # /clock 用 best-effort（sim 侧常 best_effort 发）
        clock_qos = QoSProfile(depth=10,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Clock, "/clock", self._on_clock, clock_qos)
        if HAVE_LIVOX:
            scan_qos = QoSProfile(depth=50,
                                  reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(CustomMsg, "/lidar/scan", self._on_scan, scan_qos)
        self.create_subscription(Odometry, "/state_estimation", self._on_se, 20)
        self.t0 = time.time()

    def _sim_secs(self, msg: Clock) -> float:
        return msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_clock(self, msg: Clock):
        s = self._sim_secs(msg)
        w = time.time()
        if self.sim0 is None:
            self.sim0 = s
            self.wall0 = w
        self.sim1 = s
        self.wall1 = w

    def _on_scan(self, msg):
        w = time.time()
        if self.scan_first_wall is None:
            self.scan_first_wall = w
        self.scan_last_wall = w
        self.scan_msgs += 1
        self.scan_points += int(msg.point_num)

    def _on_se(self, msg: Odometry):
        z = msg.pose.pose.position.z
        if self.z_min is None:
            self.z_min = z
            self.z_max = z
        else:
            self.z_min = min(self.z_min, z)
            self.z_max = max(self.z_max, z)
        self.z_samples += 1

    def done(self) -> bool:
        return (time.time() - self.t0) >= self.window_s

    def result(self) -> dict:
        rtf = None
        d_sim = None
        d_wall = None
        if self.sim0 is not None and self.sim1 is not None and self.wall1 is not None:
            d_sim = self.sim1 - self.sim0
            d_wall = self.wall1 - self.wall0
            rtf = (d_sim / d_wall) if d_wall > 0 else None
        # 雷达率/点率用消息本身首末墙钟跨度（比固定窗更准）
        scan_span = None
        scan_hz = None
        scan_ptrate = None
        if self.scan_first_wall is not None and self.scan_last_wall is not None \
                and self.scan_msgs > 1:
            scan_span = self.scan_last_wall - self.scan_first_wall
            if scan_span > 0:
                scan_hz = (self.scan_msgs - 1) / scan_span
                scan_ptrate = self.scan_points / scan_span
        z_pp = None
        if self.z_min is not None and self.z_max is not None:
            z_pp = self.z_max - self.z_min
        return {
            "window_s": self.window_s,
            "rtf": rtf,
            "d_sim_s": d_sim,
            "d_wall_s": d_wall,
            "scan_msgs": self.scan_msgs,
            "scan_hz": scan_hz,
            "scan_point_rate": scan_ptrate,
            "scan_span_s": scan_span,
            "have_livox": HAVE_LIVOX,
            "slam_z_min": self.z_min,
            "slam_z_max": self.z_max,
            "slam_z_pp": z_pp,
            "slam_z_samples": self.z_samples,
        }


def main():
    window_s = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    rclpy.init()
    node = GatesProbe(window_s)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        res = node.result()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("RESULT_JSON=" + json.dumps(res), flush=True)


if __name__ == "__main__":
    main()
