#!/usr/bin/env python3
"""Waypoint 回归测试：依次发布一组导航目标，判定到达/超时。

跑在 navstack 容器（source install 后）。判定基于 /state_estimation（SLAM 位姿）；
Isaac 侧地面真值由 warehouse_nav.py 的 [POSE] 日志独立记录，可交叉核对。
用法: python3 waypoint_regression.py [--tol 0.6] [--timeout 420]
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

WAYPOINTS = [(2.0, 0.0), (2.0, -2.0), (0.0, -2.0), (0.0, 0.0)]  # 方形巡航


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tol", type=float, default=0.6, help="到达判定半径 m")
    ap.add_argument("--timeout", type=float, default=420, help="单点超时（墙钟秒）")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("waypoint_regression")
    pose = {}

    def on_odom(msg: Odometry):
        p = msg.pose.pose.position
        pose["xy"] = (p.x, p.y)

    node.create_subscription(Odometry, "/state_estimation", on_odom, 5)
    wp_pub = node.create_publisher(PointStamped, "/way_point", 5)

    # 断点续跑：从自己的日志里恢复已完成的点（配合外层重生壳）
    import os
    done = set()
    if os.path.exists("/ws/regression.log"):
        for line in open("/ws/regression.log"):
            if "REACHED" in line and line.startswith("[WP"):
                done.add(int(line[3:line.index("]")]))
    if done:
        print(f"[RESUME] skipping already-reached: {sorted(done)}", flush=True)

    results = []
    for i, (gx, gy) in enumerate(WAYPOINTS):
        if i in done:
            continue
        wp = PointStamped()
        wp.header.frame_id = "map"
        wp.point.x, wp.point.y = float(gx), float(gy)
        t0 = time.time()
        reached = False
        last_pub = 0.0
        while time.time() - t0 < args.timeout:
            if time.time() - last_pub > 10.0:  # 周期重发（栈只认最新目标）
                wp_pub.publish(wp)
                last_pub = time.time()
            rclpy.spin_once(node, timeout_sec=0.2)
            if "xy" in pose:
                d = math.hypot(pose["xy"][0] - gx, pose["xy"][1] - gy)
                if d < args.tol:
                    reached = True
                    break
        dt = time.time() - t0
        cur = pose.get("xy", (float("nan"),) * 2)
        results.append((i, gx, gy, reached, dt, cur))
        print(f"[WP{i}] goal=({gx},{gy}) {'REACHED' if reached else 'TIMEOUT'} "
              f"in {dt:.0f}s wall, at=({cur[0]:.2f},{cur[1]:.2f})", flush=True)

    ok = sum(1 for r in results if r[3])
    print(f"[REGRESSION] {ok}/{len(WAYPOINTS)} reached", flush=True)


if __name__ == "__main__":
    main()
