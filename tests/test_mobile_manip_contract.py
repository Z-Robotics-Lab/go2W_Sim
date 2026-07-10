import json
import math
import re
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SIM_SCRIPTS = ROOT / "scripts/sim"
sys.path.insert(0, str(SIM_SCRIPTS))

from manip_scene import SceneConfigError, load_manip_scene  # noqa: E402
from piper_trajectory import (  # noqa: E402
    ARM_JOINT_NAMES,
    JointTrajectoryBuffer,
    TrajectoryValidationError,
)


WAREHOUSE = SIM_SCRIPTS / "warehouse_nav.py"
CAMERA = SIM_SCRIPTS / "wrist_camera.py"
BRINGUP = ROOT / "scripts/nav/bringup.sh"
SCENE_CONFIG = ROOT / "configs/manip_office_scene.json"


def make_buffer() -> JointTrajectoryBuffer:
    return JointTrajectoryBuffer(
        ARM_JOINT_NAMES,
        {name: (-3.0, 3.0) for name in ARM_JOINT_NAMES},
        {name: 2.0 for name in ARM_JOINT_NAMES},
    )


class MobileManipSceneContractTest(unittest.TestCase):
    def setUp(self):
        self.scene = load_manip_scene(
            SCENE_CONFIG,
            {"ISAAC_NUCLEUS_DIR": "https://assets.example/Isaac"},
        )

    def test_office_fixture_is_external_and_every_shelf_part_collides(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        self.assertIn("GO2W_MANIP_SCENE_CONFIG", source)
        self.assertIn("load_manip_scene", source)
        self.assertIn(
            '-e GO2W_MANIP_SCENE_CONFIG=',
            BRINGUP.read_text(encoding="utf-8"),
        )
        self.assertNotIn("003_cracker_box.usd", source)
        self.assertGreaterEqual(len(self.scene["shelf"]["parts"]), 5)
        self.assertTrue(all(part["static_collision"] for part in self.scene["shelf"]["parts"]))
        self.assertIn("shelf_surface", {part["name"] for part in self.scene["shelf"]["parts"]})
        self.assertIn("shelf_back", {part["name"] for part in self.scene["shelf"]["parts"]})

    def test_six_diverse_graspable_objects_include_handle_and_irregular_shapes(self):
        objects = self.scene["objects"]
        families = {obj["shape_family"] for obj in objects}
        self.assertGreaterEqual(len(objects), 5)
        self.assertGreaterEqual(len(families), 5)
        self.assertTrue({"box", "cylinder", "bottle"}.issubset(families))
        self.assertTrue("handled_vessel" in families)
        self.assertTrue(any("irregular" in family for family in families))
        self.assertTrue(all(obj["mass_kg"] > 0.0 for obj in objects))
        self.assertTrue(all(
            obj["physics_mode"] == "asset" or obj["collision_proxies"]
            for obj in objects
        ))

    def test_scene_loader_rejects_noncolliding_shelf(self):
        raw = json.loads(SCENE_CONFIG.read_text(encoding="utf-8"))
        raw["shelf"]["parts"][0]["static_collision"] = False
        temp = ROOT / "tests/.invalid_manip_scene.json"
        try:
            temp.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(SceneConfigError):
                load_manip_scene(temp, {"ISAAC_NUCLEUS_DIR": "x"})
        finally:
            temp.unlink(missing_ok=True)


class PiperExecutionContractTest(unittest.TestCase):
    def test_bridge_exposes_generic_execution_topics_without_gt_driven_grasp(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        for topic in (
            "/piper/joint_trajectory",
            "/piper/gripper_aperture",
            "/piper/cancel",
            "/piper/execution_status",
        ):
            self.assertIn(f'"{topic}"', source)
        self.assertNotIn("grasp.start", source)
        self.assertNotIn("PiperGraspController", source)
        self.assertNotIn("from piper_grasp", source)
        self.assertNotIn('create_subscription(String, "/piper/grasp_cmd"', source)

    def test_camera_tf_uses_navigation_body_frame(self):
        text = CAMERA.read_text(encoding="utf-8")
        match = re.search(r'GO2W_CAM_TF_PARENT",\s*"([^"]+)"', text)
        self.assertIsNotNone(match)
        self.assertEqual(match.group(1), "base_link")

    def test_carry_pose_keeps_gripper_closed(self):
        text = CAMERA.read_text(encoding="utf-8")
        carry = re.search(r'"CARRY":\s*\{([^}]+)\}', text, flags=re.DOTALL)
        self.assertIsNotNone(carry)
        self.assertIn('"piper_joint7": 0.0', carry.group(1))
        self.assertIn('"piper_joint8": 0.0', carry.group(1))

    def test_reordered_joint_names_interpolate_in_canonical_order_on_sim_time(self):
        buffer = make_buffer()
        names = tuple(reversed(ARM_JOINT_NAMES))
        final = tuple(0.1 * (i + 1) for i in range(6))
        buffer.submit(
            names,
            [tuple(0.0 for _ in names), tuple(reversed(final))],
            [0.0, 1.0],
            [0.0] * 6,
            sim_time=12.0,
        )
        halfway = buffer.sample(12.5)
        for actual, expected in zip(halfway.positions, final):
            self.assertAlmostEqual(actual, expected * 0.5)
        self.assertFalse(halfway.done)
        done = buffer.sample(13.0)
        self.assertTrue(done.done)
        self.assertEqual(buffer.status, "succeeded")

    def test_trajectory_rejects_bad_names_limits_timing_and_velocity(self):
        cases = [
            (ARM_JOINT_NAMES[:-1], [[0.0] * 5], [1.0]),
            (ARM_JOINT_NAMES, [[0.0] * 6, [0.1] * 6], [1.0, 1.0]),
            (ARM_JOINT_NAMES, [[4.0] + [0.0] * 5], [1.0]),
            (ARM_JOINT_NAMES, [[1.0] + [0.0] * 5], [0.1]),
        ]
        for names, positions, times in cases:
            with self.subTest(names=names, times=times, positions=positions):
                with self.assertRaises(TrajectoryValidationError):
                    make_buffer().submit(names, positions, times, [0.0] * 6, 0.0)

    def test_cancel_holds_measured_state(self):
        buffer = make_buffer()
        buffer.submit(
            ARM_JOINT_NAMES,
            [[0.0] * 6, [0.2] * 6],
            [0.0, 1.0],
            [0.0] * 6,
            0.0,
        )
        measured = [0.03 * i for i in range(6)]
        buffer.cancel(measured)
        self.assertEqual(buffer.status, "canceled")
        self.assertEqual(buffer.sample(100.0).positions, tuple(measured))


if __name__ == "__main__":
    unittest.main()
