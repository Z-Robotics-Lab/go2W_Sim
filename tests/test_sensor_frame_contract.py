"""Pure-logic tests for the raw IMU frame contract (P2.1, GO2W_IMU_ROUTE=raw).

The ``raw`` route must pass acc/gyr through UNCHANGED (they stay in the 20°-pitched
sensor frame for ARISE to gravity-align downstream).  Any re-rotation here would
double-compensate against the navstack extrinsic and tilt the SLAM map.  These
tests pin the passthrough contract and the warehouse_nav.py wiring that guards
against double compensation.
"""

import ast
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SIM_SCRIPTS = ROOT / "scripts/sim"
sys.path.insert(0, str(SIM_SCRIPTS))

from sensor_frame_contract import to_navigation_imu  # noqa: E402


WAREHOUSE = SIM_SCRIPTS / "warehouse_nav.py"
SYNC = ROOT / "scripts/nav/sync_navstack_files.sh"


class RawFrameContractTest(unittest.TestCase):
    def test_passthrough_preserves_values(self):
        acc = [0.12, -3.4, 9.2]
        gyr = [0.01, 0.02, -0.03]
        out_acc, out_gyr = to_navigation_imu(acc, gyr)
        self.assertEqual(out_acc, tuple(acc))
        self.assertEqual(out_gyr, tuple(gyr))

    def test_returns_tuples_not_aliased_lists(self):
        acc = [1.0, 2.0, 3.0]
        out_acc, _ = to_navigation_imu(acc, [0.0, 0.0, 0.0])
        self.assertIsInstance(out_acc, tuple)
        acc[0] = 99.0  # mutating the input must not change the emitted value
        self.assertEqual(out_acc[0], 1.0)


class ImuRouteWiringTest(unittest.TestCase):
    def test_warehouse_nav_has_mutually_exclusive_route_switch(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        ast.parse(source)
        self.assertIn('IMU_ROUTE = _os.environ.get("GO2W_IMU_ROUTE", "rotate")', source)
        self.assertIn('LIDAR_FRAME_ID = "sensor" if IMU_ROUTE == "rotate" else "mid360_raw"', source)
        self.assertIn("import sensor_frame_contract", source)
        # The raw branch must call the passthrough contract, not re-rotate.
        raw_start = source.index('else:', source.index('if IMU_ROUTE == "rotate":'))
        raw_end = source.index("imu_msg.header.stamp", raw_start)
        raw_block = source[raw_start:raw_end]
        self.assertIn("sensor_frame_contract.to_navigation_imu", raw_block)
        self.assertNotIn("Q_RY20", raw_block)         # no orientation double-rotate
        self.assertNotIn("root_quat_w", raw_block)     # no trunk re-projection

    def test_sync_navstack_zeroes_extrinsic_on_raw(self):
        sync = SYNC.read_text(encoding="utf-8")
        self.assertIn('if [ "${GO2W_IMU_ROUTE:-rotate}" = "raw" ]; then', sync)
        self.assertIn("imu_laser_rotation_offset 归零", sync)


if __name__ == "__main__":
    unittest.main()
