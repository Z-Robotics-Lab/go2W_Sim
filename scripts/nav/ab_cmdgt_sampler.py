#!/usr/bin/env python3
"""A/B 死区判据采样器：同步记录 /cmd_vel (50Hz) 与 /ground_truth/pose (5Hz)。
每收到一条 cmd_vel 就落一行（附最近一条 GT），供离线抽"真零指令子窗"比较
ON vs OFF 的段内漂移。判据在宿主侧算。

用法：python3 ab_cmdgt_sampler.py <out_csv> <duration_s>
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped


class ABSampler(Node):
    def __init__(self, out_csv: str, duration_s: float):
        super().__init__("ab_cmdgt_sampler")
        self.duration_s = duration_s
        self.gt = (float("nan"), float("nan"), float("nan"), 0.0)
        self.create_subscription(TwistStamped, "/cmd_vel", self._on_cmd, 50)
        self.create_subscription(PoseStamped, "/ground_truth/pose", self._on_gt, 10)
        self.f = open(out_csv, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["wall", "cmd_stamp", "vx", "vy", "wz",
                         "gt_stamp", "gt_x", "gt_y", "gt_z"])
        self.n = 0
        self.t0 = time.time()

    def _on_gt(self, m: PoseStamped):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.gt = (m.pose.position.x, m.pose.position.y, m.pose.position.z, st)

    def _on_cmd(self, m: TwistStamped):
        now = time.time()
        if now - self.t0 >= self.duration_s:
            self.f.flush()
            self.f.close()
            print(f"[AB] done: {self.n} cmd rows over {now - self.t0:.1f}s")
            rclpy.shutdown()
            return
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        g = self.gt
        self.w.writerow([f"{now:.3f}", f"{st:.3f}",
                         f"{m.twist.linear.x:.4f}", f"{m.twist.linear.y:.4f}",
                         f"{m.twist.angular.z:.4f}",
                         f"{g[3]:.3f}", f"{g[0]:.5f}", f"{g[1]:.5f}", f"{g[2]:.5f}"])
        self.n += 1


def main():
    rclpy.init()
    node = ABSampler(sys.argv[1], float(sys.argv[2]))
    rclpy.spin(node)


if __name__ == "__main__":
    main()
