#!/usr/bin/env python3
"""pathDir 振荡诊断采样器（H-A/H-B 弹药）。

同步记录 localPlanner 的输入与输出，供离线量化"pathDir 抖动谱 + 与 SLAM 抖动的相关性"：
  - /state_estimation (SLAM odom, Odometry) 全率：x,y,z,yaw   —— H-A 输入
  - /path (Path, vehicle 系) 每条：首段 0.5m 平均方位角 pathdir、pathSize、首点/末点  —— 输出
  - /cmd_vel (TwistStamped) 全率：vx,vy,wz                    —— 观测的振荡

每条消息各落一行（type 列区分 slam/path/cmd），带 wall 与 header.stamp。离线对齐用 stamp。
只订阅、只读；不发布、不改任何东西。

用法（navstack 容器内）：python3 pathdir_sampler.py <out_csv> <duration_s>
"""
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path


def yaw_from_quat(q) -> float:
    # ZYX yaw from quaternion
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class PathDirSampler(Node):
    def __init__(self, out_csv: str, duration_s: float):
        super().__init__("pathdir_sampler")
        self.duration_s = duration_s
        self.f = open(out_csv, "w", newline="")
        self.w = csv.writer(self.f)
        # 统一宽表：type 区分行来源；未用列留空。
        self.w.writerow([
            "wall", "type", "stamp",
            "slam_x", "slam_y", "slam_z", "slam_yaw",
            "path_size", "pathdir_first05", "path_first_x", "path_first_y",
            "path_end_x", "path_end_y",
            "vx", "vy", "wz",
        ])
        self.n = 0
        self.t0 = time.time()
        self.create_subscription(Odometry, "/state_estimation", self._on_slam, 20)
        self.create_subscription(Path, "/path", self._on_path, 20)
        self.create_subscription(TwistStamped, "/cmd_vel", self._on_cmd, 50)

    def _stop_if_done(self) -> bool:
        if time.time() - self.t0 >= self.duration_s:
            self.f.flush()
            self.f.close()
            print(f"[pathdir] done: {self.n} rows over {time.time()-self.t0:.1f}s wall")
            rclpy.shutdown()
            return True
        return False

    def _on_slam(self, m: Odometry):
        if self._stop_if_done():
            return
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        yaw = yaw_from_quat(m.pose.pose.orientation)
        self.w.writerow([f"{time.time():.4f}", "slam", f"{st:.4f}",
                         f"{p.x:.5f}", f"{p.y:.5f}", f"{p.z:.5f}", f"{yaw:.5f}",
                         "", "", "", "", "", "", "", "", ""])
        self.n += 1

    def _on_path(self, m: Path):
        if self._stop_if_done():
            return
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        n = len(m.poses)
        # 首段方向：取 path 起点到"沿路径累计弧长 >= 0.5m 处"的方位角平均。
        # path 在 vehicle 系，起点≈(0,0)。累计到 0.5m 或用完点。
        pdir = float("nan")
        fx = fy = ex = ey = float("nan")
        if n >= 1:
            fx = m.poses[0].pose.position.x
            fy = m.poses[0].pose.position.y
            ex = m.poses[n - 1].pose.position.x
            ey = m.poses[n - 1].pose.position.y
        if n >= 2:
            # 沿弧长找 0.5m 处的点
            acc = 0.0
            tx = m.poses[n - 1].pose.position.x
            ty = m.poses[n - 1].pose.position.y
            px = m.poses[0].pose.position.x
            py = m.poses[0].pose.position.y
            for i in range(1, n):
                cx = m.poses[i].pose.position.x
                cy = m.poses[i].pose.position.y
                acc += math.hypot(cx - px, cy - py)
                px, py = cx, cy
                if acc >= 0.5:
                    tx, ty = cx, cy
                    break
            pdir = math.atan2(ty - fy, tx - fx)
        self.w.writerow([f"{time.time():.4f}", "path", f"{st:.4f}",
                         "", "", "", "",
                         str(n), f"{pdir:.5f}", f"{fx:.5f}", f"{fy:.5f}",
                         f"{ex:.5f}", f"{ey:.5f}", "", "", ""])
        self.n += 1

    def _on_cmd(self, m: TwistStamped):
        if self._stop_if_done():
            return
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t = m.twist
        self.w.writerow([f"{time.time():.4f}", "cmd", f"{st:.4f}",
                         "", "", "", "", "", "", "", "", "", "",
                         f"{t.linear.x:.4f}", f"{t.linear.y:.4f}", f"{t.angular.z:.4f}"])
        self.n += 1


def main():
    rclpy.init()
    node = PathDirSampler(sys.argv[1], float(sys.argv[2]))
    rclpy.spin(node)


if __name__ == "__main__":
    main()
