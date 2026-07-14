"""Unit tests for the SLAM ground-tilt probe math (P2.4).

The probe's plane-fit / tilt / pitch math is ROS-free and lives in pure
functions so it can be verified against synthetic point clouds without a sim.
"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SIM_SCRIPTS = ROOT / "scripts/sim"
sys.path.insert(0, str(SIM_SCRIPTS))

from probe_slam_tilt import (  # noqa: E402
    fit_ground_normal,
    normal_to_tilt,
    quat_to_pitch_deg,
    filter_near_robot,
)


def _floor(n=400, z=-0.5, pitch_deg=0.0, seed=1, noise=0.0):
    """Synthesise a planar floor point cloud, optionally pitched about +Y."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-3.0, 3.0, (n, 2))
    th = math.radians(pitch_deg)
    # a plane pitched by th about Y: z = z0 + tan(th) * x
    zc = z + math.tan(th) * xy[:, 0]
    if noise:
        zc = zc + rng.normal(0.0, noise, n)
    return np.column_stack([xy, zc])


class FitGroundNormalTest(unittest.TestCase):
    def test_level_floor_is_zero_tilt(self):
        n, inl, ng = fit_ground_normal(_floor())
        self.assertIsNotNone(n)
        self.assertGreater(inl, 0.95)
        self.assertEqual(ng, 400)
        tilt, pitch, roll = normal_to_tilt(n)
        self.assertLess(tilt, 0.5)

    def test_pitched_floor_recovers_pitch_angle(self):
        n, inl, ng = fit_ground_normal(_floor(pitch_deg=15.0))
        self.assertIsNotNone(n)
        tilt, pitch, roll = normal_to_tilt(n)
        self.assertAlmostEqual(tilt, 15.0, delta=1.0)
        self.assertAlmostEqual(abs(pitch), 15.0, delta=1.0)
        self.assertAlmostEqual(roll, 0.0, delta=1.0)

    def test_noisy_floor_still_fits_within_threshold(self):
        n, inl, ng = fit_ground_normal(_floor(noise=0.01))
        self.assertIsNotNone(n)
        tilt, _, _ = normal_to_tilt(n)
        self.assertLess(tilt, 1.5)

    def test_too_few_ground_points_returns_none(self):
        pts = _floor(n=10)
        n, inl, ng = fit_ground_normal(pts)
        self.assertIsNone(n)
        self.assertEqual(inl, 0.0)

    def test_wall_only_cloud_is_rejected_as_ground(self):
        # A vertical wall (points span x=const plane) must not pass as a floor.
        rng = np.random.default_rng(3)
        yz = rng.uniform(-2.0, 2.0, (400, 2))
        wall = np.column_stack([np.zeros(400), yz[:, 0], yz[:, 1]])
        # constrain into the z-band the fitter searches; the near-vertical
        # normal (nz≈0 < 0.6) must be rejected -> None.
        wall = wall[(wall[:, 2] > -2.0) & (wall[:, 2] < 0.4)]
        n, inl, ng = fit_ground_normal(wall)
        self.assertIsNone(n)

    def test_bad_shape_raises(self):
        with self.assertRaises(ValueError):
            fit_ground_normal(np.zeros((5, 2)))


class NormalToTiltTest(unittest.TestCase):
    def test_up_normal_is_level(self):
        tilt, pitch, roll = normal_to_tilt([0.0, 0.0, 1.0])
        self.assertEqual(tilt, 0.0)
        self.assertEqual(pitch, 0.0)
        self.assertEqual(roll, 0.0)

    def test_pitch_and_roll_components_signed(self):
        # normal tilted toward +x -> positive pitch component
        _, pitch, _ = normal_to_tilt([math.sin(math.radians(10)), 0.0,
                                      math.cos(math.radians(10))])
        self.assertAlmostEqual(pitch, 10.0, delta=0.1)
        # normal tilted toward +y -> positive roll component
        _, _, roll = normal_to_tilt([0.0, math.sin(math.radians(7)),
                                     math.cos(math.radians(7))])
        self.assertAlmostEqual(roll, 7.0, delta=0.1)


class QuatToPitchTest(unittest.TestCase):
    def test_identity_is_zero_pitch(self):
        self.assertAlmostEqual(quat_to_pitch_deg(1.0, 0.0, 0.0, 0.0), 0.0, delta=1e-9)

    def test_ry20_quaternion_is_20_deg(self):
        # Ry(+20 deg) wxyz = (cos10, 0, sin10, 0) — the sim's Q_RY20.
        self.assertAlmostEqual(
            quat_to_pitch_deg(0.9848077530122081, 0.0, 0.17364817766693033, 0.0),
            20.0, delta=0.01)

    def test_gimbal_lock_is_clamped(self):
        # Ry(90 deg) -> pitch clamps to +90, no domain error.
        w = math.cos(math.radians(45))
        y = math.sin(math.radians(45))
        self.assertAlmostEqual(quat_to_pitch_deg(w, 0.0, y, 0.0), 90.0, delta=1e-6)


class NearRobotFilterTest(unittest.TestCase):
    def test_keeps_only_points_within_radius(self):
        pts = np.array([[0.0, 0.0, -0.5],   # at robot
                        [1.0, 0.0, -0.5],   # 1 m away (kept)
                        [5.0, 0.0, -0.5]])  # 5 m away (dropped)
        near = filter_near_robot(pts, (0.0, 0.0), near=3.0)
        self.assertEqual(near.shape[0], 2)

    def test_none_pose_passes_everything_through(self):
        pts = np.zeros((4, 3))
        self.assertEqual(filter_near_robot(pts, None, near=3.0).shape[0], 4)


if __name__ == "__main__":
    unittest.main()
