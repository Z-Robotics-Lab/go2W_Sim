#!/usr/bin/env python3
"""SLAM ground-tilt probe (P2.4, deploy A/B measurement instrument).

Measures the tilt of the SLAM map's ground plane relative to gravity, for the
P2 IMU-route / bringup-order A/B adjudication (docs/stability-gates.md).

What it does, purely by READING:
  1. Subscribes to a registered point cloud already expressed in the SLAM map
     frame (default ``/registered_scan``) plus the robot pose (default
     ``/state_estimation``), accumulates points over a bounded window, keeps only
     ground points in a band NEAR the robot in XY, and fits the dominant
     near-horizontal plane by a gravity-seeded RANSAC.
  2. Reports the angle between the fitted ground normal and world +Z (the SLAM
     map tilt), decomposed into pitch/roll components.
  3. Simultaneously captures ARISE's own IMU pitch estimate from the published
     IMU orientation (default ``/imu/data``), so the map tilt can be compared
     against what the localizer thinks its pitch is.

READ-ONLY: subscribes only; never publishes, never touches the sim. Safe to run
against a live navstack held by another session. Bounded ``--secs`` then exits;
result is printed as a single JSON line (``RESULT_JSON: {...}``) for the A/B
harness plus a human-readable summary line.

The plane-fit / tilt math lives in pure, ROS-free functions
(``fit_ground_normal``, ``normal_to_tilt``, ``quat_to_pitch_deg``) so it is unit
tested against synthetic point clouds without a sim.

Usage (inside navstack container, ROS sourced):
  python3 probe_slam_tilt.py [--cloud /registered_scan] [--pose /state_estimation]
                             [--imu /imu/data] [--secs 12] [--near 3.0]
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import sys

import numpy as np


# --------------------------------------------------------------------------- #
# Pure math (ROS-free, unit tested)                                            #
# --------------------------------------------------------------------------- #
def fit_ground_normal(pts, z_lo=-2.0, z_hi=0.4, iters=400, thresh=0.05, seed=0):
    """Gravity-seeded RANSAC on low points.

    Returns ``(normal_unit | None, inlier_frac, n_ground)``.  The normal is
    oriented to point up (+Z hemisphere).  Near-vertical candidate planes (walls)
    are rejected so a wall never masquerades as the floor.
    """
    pts = np.asarray(pts, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("pts must be an (N,3) array")
    band = pts[(pts[:, 2] > z_lo) & (pts[:, 2] < z_hi)]
    if band.shape[0] < 50:
        return None, 0.0, int(band.shape[0])
    rng = np.random.default_rng(seed)
    best_n, best_inl = None, -1
    n = band.shape[0]
    for _ in range(iters):
        idx = rng.choice(n, 3, replace=False)
        p0, p1, p2 = band[idx]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(nrm)
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        if nrm[2] < 0:
            nrm = -nrm
        if nrm[2] < 0.6:  # a floor must be roughly horizontal
            continue
        d = -nrm.dot(p0)
        dist = np.abs(band.dot(nrm) + d)
        inl = int((dist < thresh).sum())
        if inl > best_inl:
            best_inl, best_n = inl, nrm
    if best_n is None:
        return None, 0.0, int(band.shape[0])
    return best_n, best_inl / n, int(band.shape[0])


def normal_to_tilt(normal):
    """Decompose an up-oriented ground normal into tilt angles (degrees).

    Returns ``(tilt_deg, pitch_component_deg, roll_component_deg)`` where
    ``tilt_deg`` is the total angle to world +Z.
    """
    nrm = np.asarray(normal, dtype=np.float64)
    nz = float(np.clip(nrm[2], -1.0, 1.0))
    tilt = math.degrees(math.acos(nz))
    pitch = math.degrees(math.atan2(nrm[0], nrm[2]))
    roll = math.degrees(math.atan2(nrm[1], nrm[2]))
    return tilt, pitch, roll


def quat_to_pitch_deg(w, x, y, z):
    """Pitch (rotation about body Y, degrees) from a wxyz quaternion.

    Uses the standard ZYX-Euler pitch extraction with the +-90 deg clamp so a
    gimbal-lock singularity does not blow up.
    """
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.degrees(math.asin(sinp))


# --------------------------------------------------------------------------- #
# ROS plumbing (import guarded so the math stays importable without rclpy)     #
# --------------------------------------------------------------------------- #
def _read_xyz(msg):
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


def filter_near_robot(pts, robot_xy, near):
    """Keep points within ``near`` metres of the robot in XY (ground under it)."""
    if robot_xy is None:
        return pts
    dx = pts[:, 0] - robot_xy[0]
    dy = pts[:, 1] - robot_xy[1]
    return pts[(dx * dx + dy * dy) <= (near * near)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cloud", default="/registered_scan",
                    help="registered point cloud topic in the SLAM map frame")
    ap.add_argument("--pose", default="/state_estimation",
                    help="robot pose odometry topic (for near-robot filtering)")
    ap.add_argument("--imu", default="/imu/data",
                    help="IMU topic carrying ARISE's orientation estimate")
    ap.add_argument("--secs", type=float, default=12.0)
    ap.add_argument("--near", type=float, default=3.0,
                    help="keep ground points within this XY radius of the robot")
    args = ap.parse_args(argv)
    return _spin(args)


def _spin(args):  # pragma: no cover - requires a live ROS graph
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2

    class Probe(Node):
        def __init__(self):
            super().__init__("probe_slam_tilt")
            self.acc = []
            self.robot_xy = None
            self.imu_pitch = None
            self.done = False
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(PointCloud2, args.cloud, self._cloud_cb, qos)
            self.create_subscription(Odometry, args.pose, self._pose_cb, qos)
            self.create_subscription(Imu, args.imu, self._imu_cb, qos)
            self.t0 = self.get_clock().now()
            self.get_logger().info(
                f"probing cloud={args.cloud} pose={args.pose} imu={args.imu} "
                f"for {args.secs}s (near={args.near}m) ...")

        def _cloud_cb(self, msg):
            xyz = _read_xyz(msg)
            if xyz.size:
                self.acc.append(xyz)
            self._maybe_finish()

        def _pose_cb(self, msg):
            p = msg.pose.pose.position
            self.robot_xy = (p.x, p.y)

        def _imu_cb(self, msg):
            q = msg.orientation
            self.imu_pitch = quat_to_pitch_deg(q.w, q.x, q.y, q.z)

        def _maybe_finish(self):
            if not self.done and \
                    (self.get_clock().now() - self.t0).nanoseconds / 1e9 > args.secs:
                self.done = True
                self._emit()

        def _emit(self):
            result = {"cloud": args.cloud, "near_m": args.near,
                      "arise_imu_pitch_deg": self.imu_pitch,
                      "robot_xy": self.robot_xy}
            if not self.acc:
                result["verdict"] = "NO_POINTS"
            else:
                pts = np.concatenate(self.acc, 0)
                near = filter_near_robot(pts, self.robot_xy, args.near)
                nrm, inl, ng = fit_ground_normal(near)
                result["npts"] = int(pts.shape[0])
                result["near_npts"] = int(near.shape[0])
                result["ground_band"] = ng
                if nrm is None:
                    result["verdict"] = "NO_PLANE"
                else:
                    tilt, pitch, roll = normal_to_tilt(nrm)
                    result.update(
                        verdict="LEVEL" if tilt < 2.0 else "TILTED",
                        ground_tilt_deg=round(tilt, 3),
                        pitch_component_deg=round(pitch, 3),
                        roll_component_deg=round(roll, 3),
                        inlier_frac=round(inl, 3),
                        normal=[round(float(v), 4) for v in nrm])
            print("RESULT_JSON: " + json.dumps(result), flush=True)
            print(f"RESULT: {result.get('verdict')} "
                  f"ground_tilt={result.get('ground_tilt_deg')}deg "
                  f"arise_imu_pitch={result.get('arise_imu_pitch_deg')}deg", flush=True)
            rclpy.shutdown()

    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
