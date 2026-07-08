#!/usr/bin/env python3
"""/terrain_map 阈值闪烁探针(fix-c/fix-a 诊断弹药;重造——c 轮原件未落盘,接口按 DEBUG.md 描述复原)。

逐帧统计 intensity(=disZ 地形代价)超各档门的 cell 数,全局 + 前扇区(vehicle +x 0.2-2m,|y|<1m,
用 /state_estimation 位姿把 map 系 cell 转到车体系)。闪烁=cell 数时序 std。只订阅只读。

用法(navstack 容器内): python3 probe_terrain_map.py <n_frames> [out_txt]
"""
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


GATES = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30)


class Probe(Node):
    def __init__(self, n_frames):
        super().__init__("terrain_probe")
        self.n = n_frames
        self.frames = []          # per frame: {gate: (global_cnt, front_cnt)}
        self.front_max = []       # per frame max intensity in front sector
        self.pose = None
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, 10)
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_map, 10)

    def cb_odom(self, m):
        p = m.pose.pose
        siny = 2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y)
        cosy = 1.0 - 2.0 * (p.orientation.y ** 2 + p.orientation.z ** 2)
        self.pose = (p.position.x, p.position.y, math.atan2(siny, cosy))

    def cb_map(self, m):
        if self.pose is None or len(self.frames) >= self.n:
            return
        px, py, yaw = self.pose
        c, s = math.cos(-yaw), math.sin(-yaw)
        counts = {g: [0, 0] for g in GATES}
        fmax = 0.0
        for x, y, _z, inten in pc2.read_points(m, ("x", "y", "z", "intensity"), skip_nans=True):
            vx = (x - px) * c - (y - py) * s
            vy = (x - px) * s + (y - py) * c
            in_front = 0.2 <= vx <= 2.0 and abs(vy) < 1.0
            if in_front and inten > fmax:
                fmax = inten
            for g in GATES:
                if inten > g:
                    counts[g][0] += 1
                    if in_front:
                        counts[g][1] += 1
        self.frames.append(counts)
        self.front_max.append(fmax)


def std(v):
    if len(v) < 2:
        return 0.0
    mu = sum(v) / len(v)
    return (sum((x - mu) ** 2 for x in v) / len(v)) ** 0.5


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    out = sys.argv[2] if len(sys.argv) > 2 else None
    rclpy.init()
    node = Probe(n)
    while len(node.frames) < n and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
    lines = [f"frames={len(node.frames)}  front max-intensity min/max="
             f"{min(node.front_max):.3f}/{max(node.front_max):.3f}"]
    for g in GATES:
        glob = [f[g][0] for f in node.frames]
        front = [f[g][1] for f in node.frames]
        lines.append(
            f">{g:.2f}: global min/max/std={min(glob)}/{max(glob)}/{std(glob):.1f}"
            f"  front min/max/std={min(front)}/{max(front)}/{std(front):.1f}")
    txt = "\n".join(lines)
    print(txt)
    if out:
        with open(out, "w") as f:
            f.write(txt + "\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
