import importlib.util
import math
import re
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


class SensorFrameContractTest(unittest.TestCase):
    def test_isaac_imu_bridge_does_not_add_mount_pitch(self):
        module_path = ROOT / "scripts/sim/sensor_frame_contract.py"
        spec = importlib.util.spec_from_file_location("sensor_frame_contract", module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        accel, gyro = module.to_navigation_imu(
            (-0.35, 0.08, 9.80),
            (0.01, -0.02, 0.03),
        )

        self.assertEqual(accel, (-0.35, 0.08, 9.80))
        self.assertEqual(gyro, (0.01, -0.02, 0.03))

    def test_measured_raw_cloud_and_imu_agree_after_duplicate_rotation_is_removed(self):
        module_path = ROOT / "scripts/sim/sensor_frame_contract.py"
        spec = importlib.util.spec_from_file_location("sensor_frame_contract", module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        angle = math.radians(20.0)
        c, s = math.cos(angle), math.sin(angle)
        old_mount_rotation = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
        old_published_gravity = np.array([3.02095254, 0.07406798, 9.33294451])
        gravity_before_old_rotation = old_mount_rotation.T @ old_published_gravity
        gravity, _ = module.to_navigation_imu(gravity_before_old_rotation, (0.0, 0.0, 0.0))

        ax, ay, az = gravity
        pitch = math.atan2(ax, math.sqrt(ay * ay + az * az))
        roll = math.atan2(-ay, az)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        gravity_attitude = np.array([
            [cp, 0.0, sp],
            [sr * sp, cr, -sr * cp],
            [-cr * sp, sr, cr * cp],
        ])

        measured_raw_ground_normal = np.array([-0.03597187, 0.00746117, 0.99932495])
        level_normal = np.linalg.inv(gravity_attitude) @ measured_raw_ground_normal
        level_normal /= np.linalg.norm(level_normal)
        residual_tilt_deg = math.degrees(math.acos(np.clip(level_normal[2], -1.0, 1.0)))
        self.assertLess(residual_tilt_deg, 0.02)

    def test_navstack_patch_uses_relative_extrinsic_only(self):
        patch = (ROOT / "scripts/nav/patch_navstack.sh").read_text()
        self.assertIn("system_real_robot.launch.py", patch)
        match = re.search(
            r"imu_laser_rotation_offset.*?data:\s*\[([^]]+)\]",
            patch,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match)
        values = [float(value.strip()) for value in match.group(1).split(",")]
        self.assertEqual(values, [0.0, 0.0, 0.0])

    def test_runtime_calibration_sync_replaces_stale_install_copy(self):
        module_path = ROOT / "scripts/nav/sync_runtime_config.py"
        spec = importlib.util.spec_from_file_location("sync_runtime_config", module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        calibration = """%YAML:1.0
imu_laser_rotation_offset: !!opencv-matrix
  rows: 3
  cols: 1
  dt: d
  data: [0.0, 0.0, 0.0]
"""
        stale = calibration.replace("[0.0, 0.0, 0.0]", "[0.0, 20.0, 0.0]")
        with tempfile.TemporaryDirectory() as temp_dir:
            nav = Path(temp_dir)
            source = nav / module.CALIBRATION_RELATIVE
            destination = nav / module.CALIBRATION_INSTALL_RELATIVE
            source.parent.mkdir(parents=True)
            destination.parent.mkdir(parents=True)
            source.write_text(calibration)
            destination.write_text(stale)

            synced = module.sync_calibration(nav)

            self.assertEqual(synced, destination)
            self.assertEqual(destination.read_text(), calibration)

    def test_bringup_waits_for_settled_isaac_before_navstack(self):
        bringup = (ROOT / "scripts/nav/bringup.sh").read_text()
        self.assertIn('until grep -qa "imu settled"', bringup)
        self.assertIn('until bash "$HERE/status.sh"', bringup)
        self.assertIn("GO2W_NAV_TIMEOUT_S", bringup)
        self.assertLess(
            bringup.index('_phase "isaac bridge (paired restart)"'),
            bringup.index('_phase "navstack supervisor (paired restart)"'),
        )

    def test_sim_dds_is_local_and_uses_unique_default_domain(self):
        bringup = (ROOT / "scripts/nav/bringup.sh").read_text()
        supervisor = (ROOT / "scripts/nav/run_all_forever.sh").read_text()
        self.assertIn('GO2W_ROS_DOMAIN_ID:-184', bringup)
        self.assertIn('GO2W_ROS_LOCALHOST_ONLY:-1', bringup)
        self.assertIn('-e ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY"', bringup)
        self.assertIn('ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-184}"', supervisor)
        self.assertIn('ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"', supervisor)
        self.assertIn('/registered_scan /state_estimation', bringup)
        self.assertIn('publisher_count=', bringup)
        self.assertIn('_runtime_dds_matches', bringup)
        self.assertIn('&& _ros_graph_unique', bringup)
        self.assertIn('ros_stream_gate.py --duration 12', bringup)


if __name__ == "__main__":
    unittest.main()
