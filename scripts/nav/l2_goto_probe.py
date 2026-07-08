#!/usr/bin/env python3
"""L2 局部导航测试驱动：发一个 SLAM 系航点，全程采样 cmd/SLAM/GT，
到点后 hold 采样漂移。诚实优先——到点判据在 SLAM 系（栈自己的估计，即真实驱动系），
但位移/实速用 GT 记录（物理真值，规避 L1 已测的 SLAM 前向 scale ~11-24% 高报）。

用法:
  l2_goto_probe.py <out_csv> <slam_x> <slam_y> [--tol 0.6] [--timeout 240]
                   [--hold 10] [--duty-thresh 0.05]

发布 /way_point (map=SLAM 系, 每 8s 重发)。到点=SLAM 系到目标 <tol 且连续 1.5s 保持。
到点后继续采样 hold 秒（漂移观测）。CSV 每收到一条 cmd_vel 落一行。
"""
import argparse
import csv
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_csv")
    ap.add_argument("slam_x", type=float)
    ap.add_argument("slam_y", type=float)
    ap.add_argument("--tol", type=float, default=0.6)
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--hold", type=float, default=10.0)
    ap.add_argument("--duty-thresh", type=float, default=0.05)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("l2_goto_probe")

    st = {"slam": None, "gt": None, "up_z": float("nan")}

    def on_slam(m: Odometry):
        p = m.pose.pose.position
        st["slam"] = (p.x, p.y, p.z)

    def on_gt(m: PoseStamped):
        p = m.pose.position
        stt = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        st["gt"] = (p.x, p.y, p.z, stt)

    def on_up(m: Float32):
        st["up_z"] = float(m.data)

    node.create_subscription(Odometry, "/state_estimation", on_slam, 5)
    node.create_subscription(PoseStamped, "/ground_truth/pose", on_gt, 10)
    node.create_subscription(Float32, "/ground_truth/up_z", on_up, 10)
    cmd = {"v": None}

    f = open(args.out_csv, "w", newline="")
    w = csv.writer(f)
    w.writerow(["wall", "phase", "vx", "vy", "wz",
                "slam_x", "slam_y", "slam_z",
                "gt_x", "gt_y", "gt_z", "gt_stamp", "up_z",
                "slam_dist_to_goal"])

    wp_pub = node.create_publisher(PointStamped, "/way_point", 5)
    wp = PointStamped()
    wp.header.frame_id = "map"
    wp.point.x, wp.point.y = args.slam_x, args.slam_y

    def on_cmd(m: TwistStamped):
        cmd["v"] = (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)
    node.create_subscription(TwistStamped, "/cmd_vel", on_cmd, 50)

    # wait for first fixes
    t_wait = time.time()
    while (st["slam"] is None or st["gt"] is None) and time.time() - t_wait < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    if st["slam"] is None or st["gt"] is None:
        print("[L2] ERROR no slam/gt fix", flush=True)
        return

    slam0 = st["slam"]
    gt0 = st["gt"]
    print(f"[L2] start SLAM=({slam0[0]:.2f},{slam0[1]:.2f}) "
          f"GT=({gt0[0]:.2f},{gt0[1]:.2f}) goal_slam=({args.slam_x:.2f},{args.slam_y:.2f})",
          flush=True)

    t0 = time.time()
    last_pub = 0.0
    reached = False
    reach_since = None
    reach_wall = None
    reach_slam = None
    reach_gt = None
    phase = "drive"

    while True:
        now = time.time()
        el = now - t0
        # republish waypoint during drive phase
        if not reached and now - last_pub > 8.0:
            wp.header.stamp = node.get_clock().now().to_msg()
            wp_pub.publish(wp)
            last_pub = now
        rclpy.spin_once(node, timeout_sec=0.05)

        s = st["slam"]; g = st["gt"]
        d = float("nan")
        if s is not None:
            d = math.hypot(s[0] - args.slam_x, s[1] - args.slam_y)
        # arrival detection (SLAM frame), require 1.5s sustained
        if not reached and s is not None and d < args.tol:
            if reach_since is None:
                reach_since = now
            elif now - reach_since > 1.5:
                reached = True
                reach_wall = now - t0
                reach_slam = s
                reach_gt = g
                phase = "hold"
                print(f"[L2] REACHED (slam) d={d:.3f} at {reach_wall:.1f}s wall; "
                      f"holding {args.hold}s", flush=True)
        elif not reached and (s is None or d >= args.tol):
            reach_since = None

        # log a row on every cmd (approx cmd rate); else ~throttle
        v = cmd["v"]
        if v is not None:
            w.writerow([f"{now:.3f}", phase,
                        f"{v[0]:.4f}", f"{v[1]:.4f}", f"{v[2]:.4f}",
                        f"{s[0]:.4f}" if s else "nan",
                        f"{s[1]:.4f}" if s else "nan",
                        f"{s[2]:.4f}" if s else "nan",
                        f"{g[0]:.5f}" if g else "nan",
                        f"{g[1]:.5f}" if g else "nan",
                        f"{g[2]:.5f}" if g else "nan",
                        f"{g[3]:.3f}" if g else "nan",
                        f"{st['up_z']:.5f}",
                        f"{d:.4f}"])
            cmd["v"] = None

        if reached and (now - t0 - reach_wall) > args.hold:
            break
        if not reached and el > args.timeout:
            print(f"[L2] TIMEOUT after {el:.0f}s, slam_dist={d:.3f}", flush=True)
            break

    f.flush(); f.close()

    # summary
    gnow = st["gt"]; snow = st["slam"]
    gt_disp = math.hypot(gnow[0] - gt0[0], gnow[1] - gt0[1]) if gnow and gt0 else float("nan")
    slam_disp = math.hypot(snow[0] - slam0[0], snow[1] - slam0[1]) if snow and slam0 else float("nan")
    print(f"[L2] SUMMARY reached={reached} timeout={not reached}", flush=True)
    print(f"[L2]   final_slam_dist_to_goal={d:.3f}", flush=True)
    print(f"[L2]   GT net disp={gt_disp:.3f}m  SLAM net disp={slam_disp:.3f}m", flush=True)
    if reach_gt and gnow:
        hold_drift = math.hypot(gnow[0] - reach_gt[0], gnow[1] - reach_gt[1])
        print(f"[L2]   post-arrival GT hold drift over {args.hold}s = {hold_drift:.3f}m", flush=True)
    print(f"[L2]   final up_z={st['up_z']:.4f}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
