#!/usr/bin/env python3
"""Ground-plane normal probe — Finding A (extrinsic tilt) verification tool.

Subscribes to a registered point cloud already expressed in the SLAM map frame
(default /registered_scan) plus /state_estimation, accumulates points over a
short window, fits the dominant near-horizontal plane by a gravity-seeded RANSAC,
and reports the angle between the fitted ground normal and world +Z.

READ-ONLY: subscribes only; never publishes, never touches the sim. Safe to run
against a live navstack held by another session.

Acceptance (Finding A): ground-normal-vs-vertical angle < 2 deg => map is level.
A persistent ~20 deg means the sim<->SLAM extrinsic disagree in sign/magnitude.

Usage (inside navstack container, ROS sourced):
  python3 ground_normal_probe.py [--topic /registered_scan] [--secs 12]
"""
import argparse
import struct
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


def _read_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract XYZ as (N,3) float32 from a PointCloud2 without pcl deps."""
    names = {f.name: f for f in msg.fields}
    for req in ("x", "y", "z"):
        if req not in names:
            return np.empty((0, 3), np.float32)
    ox, oy, oz = names["x"].offset, names["y"].offset, names["z"].offset
    step = msg.point_step
    n = msg.width * msg.height
    buf = bytes(msg.data)
    out = np.empty((n, 3), np.float32)
    for i in range(n):
        base = i * step
        out[i, 0] = struct.unpack_from("<f", buf, base + ox)[0]
        out[i, 1] = struct.unpack_from("<f", buf, base + oy)[0]
        out[i, 2] = struct.unpack_from("<f", buf, base + oz)[0]
    return out


def fit_ground_normal(pts: np.ndarray, z_lo=-2.0, z_hi=0.4, iters=400, thresh=0.05):
    """Gravity-seeded RANSAC on low points; returns (normal_unit, inlier_frac, n_ground)."""
    # keep points plausibly on/near the floor relative to sensor height band
    band = pts[(pts[:, 2] > z_lo) & (pts[:, 2] < z_hi)]
    if band.shape[0] < 50:
        return None, 0.0, band.shape[0]
    rng = np.random.default_rng(0)
    best_n, best_inl = None, -1
    N = band.shape[0]
    for _ in range(iters):
        idx = rng.choice(N, 3, replace=False)
        p0, p1, p2 = band[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        if nrm[2] < 0:
            nrm = -nrm
        # a ground plane must be roughly horizontal; reject near-vertical candidates early
        if nrm[2] < 0.6:
            continue
        d = -nrm.dot(p0)
        dist = np.abs(band.dot(nrm) + d)
        inl = int((dist < thresh).sum())
        if inl > best_inl:
            best_inl, best_n = inl, nrm
    if best_n is None:
        return None, 0.0, band.shape[0]
    # refine on inliers
    d = -best_n.dot(band[np.argmax(np.abs(band.dot(best_n)) < thresh)]) if False else 0
    return best_n, best_inl / N, band.shape[0]


class Probe(Node):
    def __init__(self, topic, secs):
        super().__init__("ground_normal_probe")
        self.acc = []
        self.secs = secs
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(PointCloud2, topic, self.cb, qos)
        self.t0 = self.get_clock().now()
        self.get_logger().info(f"probing {topic} for {secs}s ...")

    def cb(self, msg):
        xyz = _read_xyz(msg)
        if xyz.size:
            self.acc.append(xyz)
        if (self.get_clock().now() - self.t0).nanoseconds / 1e9 > self.secs:
            self.finish()

    def finish(self):
        if not self.acc:
            print("RESULT: NO_POINTS", flush=True)
            rclpy.shutdown()
            return
        pts = np.concatenate(self.acc, 0)
        nrm, inl, ng = fit_ground_normal(pts)
        if nrm is None:
            print(f"RESULT: NO_PLANE npts={pts.shape[0]} ground_band={ng}", flush=True)
            rclpy.shutdown()
            return
        ang = np.degrees(np.arccos(np.clip(nrm[2], -1, 1)))
        # decompose tilt into pitch (x) and roll (y) directions of the normal
        pitch_tilt = np.degrees(np.arctan2(nrm[0], nrm[2]))
        roll_tilt = np.degrees(np.arctan2(nrm[1], nrm[2]))
        verdict = "LEVEL(<2deg)" if ang < 2.0 else "TILTED"
        print(f"RESULT: {verdict} normal_vs_vertical={ang:.2f}deg "
              f"pitch_component={pitch_tilt:.2f} roll_component={roll_tilt:.2f} "
              f"inlier_frac={inl:.2f} npts={pts.shape[0]} ground_band={ng} "
              f"normal=[{nrm[0]:.3f},{nrm[1]:.3f},{nrm[2]:.3f}]", flush=True)
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/registered_scan")
    ap.add_argument("--secs", type=float, default=12.0)
    a = ap.parse_args()
    rclpy.init()
    node = Probe(a.topic, a.secs)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
