#!/usr/bin/env python3
"""E0' 死区验收采样器：10Hz 采 /ground_truth/pose (GT) + /state_estimation (SLAM)
到 CSV，跑 DURATION 秒。判据（分析在宿主侧做）：GT 位移 < 0.05m + 站姿 z 方差小。

用法（navstack 容器内）：
  python3 e0prime_sampler.py <out_csv> <duration_s>
"""
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class Sampler(Node):
    def __init__(self, out_csv: str, duration_s: float):
        super().__init__("e0prime_sampler")
        self.duration_s = duration_s
        self.gt = None   # (x,y,z, stamp_s)
        self.slam = None
        self.create_subscription(PoseStamped, "/ground_truth/pose", self._on_gt, 10)
        self.create_subscription(Odometry, "/state_estimation", self._on_slam, 10)
        self.f = open(out_csv, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["wall_t", "gt_stamp", "gt_x", "gt_y", "gt_z",
                         "slam_stamp", "slam_x", "slam_y", "slam_z"])
        self.n = 0
        self.t0 = time.time()
        self.create_timer(0.1, self._tick)  # 10Hz

    def _on_gt(self, m: PoseStamped):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.gt = (m.pose.position.x, m.pose.position.y, m.pose.position.z, st)

    def _on_slam(self, m: Odometry):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        self.slam = (p.x, p.y, p.z, st)

    def _tick(self):
        now = time.time()
        if now - self.t0 >= self.duration_s:
            self.f.flush()
            self.f.close()
            print(f"[E0'] done: {self.n} samples over {now - self.t0:.1f}s wall")
            rclpy.shutdown()
            return
        gt = self.gt or (float("nan"),) * 4
        sl = self.slam or (float("nan"),) * 4
        self.w.writerow([f"{now:.3f}", f"{gt[3]:.3f}", f"{gt[0]:.5f}",
                         f"{gt[1]:.5f}", f"{gt[2]:.5f}", f"{sl[3]:.3f}",
                         f"{sl[0]:.5f}", f"{sl[1]:.5f}", f"{sl[2]:.5f}"])
        self.n += 1


def main():
    out_csv = sys.argv[1]
    duration_s = float(sys.argv[2])
    rclpy.init()
    node = Sampler(out_csv, duration_s)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
