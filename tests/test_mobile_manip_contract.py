import json
import math
import re
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SIM_SCRIPTS = ROOT / "scripts/sim"
sys.path.insert(0, str(SIM_SCRIPTS))

from manip_scene import (  # noqa: E402
    legacy_grasp_box_enabled,
    SceneConfigError,
    load_manip_scene,
)
from piper_trajectory import (  # noqa: E402
    ARM_JOINT_NAMES,
    format_execution_status,
    GripperCommandBuffer,
    GripperValidationError,
    JointTrajectoryBuffer,
    TrajectoryValidationError,
)
from wrist_camera import camera_update_elapsed_dt  # noqa: E402


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

    def test_office_fixture_disables_legacy_box_in_the_approach_corridor(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        self.assertFalse(legacy_grasp_box_enabled(self.scene))
        self.assertIn(
            "box = RigidObject(BOX_CFG) if SPAWN_LEGACY_GRASP_BOX else None",
            source,
        )
        self.assertIn("SPAWN_LEGACY_GRASP_BOX = legacy_grasp_box_enabled(MANIP_SCENE)", source)

    def test_warehouse_default_and_explicit_fixture_enable_keep_legacy_box(self):
        self.assertTrue(legacy_grasp_box_enabled(None))
        raw = json.loads(SCENE_CONFIG.read_text(encoding="utf-8"))
        raw["fixtures"]["legacy_grasp_box"]["enabled"] = True
        temp = ROOT / "tests/.enabled_legacy_box_scene.json"
        try:
            temp.write_text(json.dumps(raw), encoding="utf-8")
            scene = load_manip_scene(temp, {"ISAAC_NUCLEUS_DIR": "x"})
            self.assertTrue(legacy_grasp_box_enabled(scene))
        finally:
            temp.unlink(missing_ok=True)

    def test_scene_loader_rejects_non_boolean_legacy_box_toggle(self):
        raw = json.loads(SCENE_CONFIG.read_text(encoding="utf-8"))
        raw["fixtures"]["legacy_grasp_box"]["enabled"] = "false"
        temp = ROOT / "tests/.invalid_fixture_toggle_scene.json"
        try:
            temp.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaises(SceneConfigError):
                load_manip_scene(temp, {"ISAAC_NUCLEUS_DIR": "x"})
        finally:
            temp.unlink(missing_ok=True)

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
    def test_camera_streams_use_nonblocking_sensor_qos(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        self.assertIn("from rclpy.qos import qos_profile_sensor_data", source)
        for endpoint in (
            'Image, "/camera/image", qos_profile_sensor_data',
            'Image, "/camera/depth", qos_profile_sensor_data',
            "Image, wc.TOPIC_COLOR, qos_profile_sensor_data",
            "CameraInfo, wc.TOPIC_COLOR_INFO, qos_profile_sensor_data",
            "Image, wc.TOPIC_DEPTH_ALIGNED, qos_profile_sensor_data",
        ):
            self.assertIn(endpoint, source)

    def test_health_requires_live_isaac_process_and_loop_heartbeat(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        status = (ROOT / "scripts/nav/status.sh").read_text(encoding="utf-8")
        bringup = BRINGUP.read_text(encoding="utf-8")
        self.assertIn(".isaac_heartbeat", source)
        self.assertIn("_heartbeat_fresh", status)
        self.assertIn("warehouse_nav.py", status)
        self.assertIn("执行成对 teardown 后自愈重启", bringup)

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

    def test_camera_update_advances_the_full_ros_publish_period(self):
        for stride, publish_period in ((10, 0.1), (50, 0.5)):
            with self.subTest(stride=stride):
                self.assertAlmostEqual(
                    camera_update_elapsed_dt(0.01, stride), publish_period)

        source = WAREHOUSE.read_text(encoding="utf-8")
        self.assertIn(
            "camera_update_dt = wc.camera_update_elapsed_dt(physics_dt, CAM_STRIDE)",
            source,
        )
        self.assertIn(
            "math.isclose(camera_update_dt, CAM_UPDATE_PERIOD,", source)
        self.assertIn("d435.update(camera_update_dt)", source)
        self.assertNotIn("d435.update(physics_dt)", source)

    def test_camera_update_elapsed_dt_rejects_invalid_clock_inputs(self):
        for physics_dt, stride in ((0.0, 10), (float("nan"), 10), (0.01, 0),
                                   (0.01, True)):
            with self.subTest(physics_dt=physics_dt, stride=stride):
                with self.assertRaises(ValueError):
                    camera_update_elapsed_dt(physics_dt, stride)

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
            segment="transit",
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
                    make_buffer().submit(
                        names,
                        positions,
                        times,
                        [0.0] * 6,
                        0.0,
                        segment="transit",
                    )

    def test_cancel_holds_measured_state(self):
        buffer = make_buffer()
        buffer.submit(
            ARM_JOINT_NAMES,
            [[0.0] * 6, [0.2] * 6],
            [0.0, 1.0],
            [0.0] * 6,
            0.0,
            segment="approach",
        )
        measured = [0.03 * i for i in range(6)]
        buffer.cancel(measured)
        self.assertEqual(buffer.status, "canceled")
        self.assertEqual(buffer.sample(100.0).positions, tuple(measured))

    def test_trajectory_identity_is_strictly_increasing_and_transactional(self):
        buffer = make_buffer()
        first = buffer.submit(
            ARM_JOINT_NAMES,
            [[0.0] * 6, [0.1] * 6],
            [0.0, 1.0],
            [0.0] * 6,
            2.0,
            segment="transit",
        )
        self.assertEqual(first, 1)
        self.assertEqual(buffer.command_id, 1)
        self.assertEqual(buffer.segment, "transit")
        self.assertEqual(buffer.received_at, 2.0)

        with self.assertRaises(TrajectoryValidationError):
            buffer.submit(
                ARM_JOINT_NAMES,
                [[0.0] * 6, [0.1] * 6],
                [0.0, 1.0],
                [0.0] * 6,
                3.0,
                segment="not-a-segment",
            )
        self.assertEqual(buffer.command_id, 1)
        self.assertEqual(buffer.segment, "transit")
        self.assertEqual(buffer.received_at, 2.0)

        second = buffer.submit(
            ARM_JOINT_NAMES,
            [[0.0] * 6, [0.2] * 6],
            [0.0, 1.0],
            [0.0] * 6,
            4.0,
            segment="carry",
        )
        self.assertEqual(second, 2)
        self.assertEqual(buffer.command_id, 2)
        self.assertEqual(buffer.segment, "carry")

        third = buffer.submit(
            ARM_JOINT_NAMES,
            [[0.2] * 6, [0.1] * 6],
            [0.0, 1.0],
            [0.2] * 6,
            5.0,
            segment="place_retreat",
        )
        self.assertEqual(third, 3)
        self.assertEqual(buffer.segment, "place_retreat")

    def test_gripper_identity_is_independent_and_rejection_keeps_last_command(self):
        gripper = GripperCommandBuffer(0.0, 0.07)
        first = gripper.submit(0.06, 1.25)
        self.assertEqual(first.command_id, 1)
        self.assertEqual(first.aperture, 0.06)

        with self.assertRaises(GripperValidationError):
            gripper.submit(0.08, 2.0)
        self.assertEqual(gripper.command_id, 1)
        self.assertEqual(gripper.aperture, 0.06)
        self.assertEqual(gripper.received_at, 1.25)

        gripper.reject("out of range", 2.0)
        fields = gripper.status_fields()
        self.assertIn("gripper=rejected:out_of_range", fields)
        self.assertIn("gripper_command_id=1", fields)
        self.assertIn("gripper_command_aperture=0.0600", fields)
        self.assertIn("gripper_received_at=1.250000", fields)
        self.assertIn("gripper_event_received_at=2.000000", fields)

        second = gripper.submit(0.04, 3.0)
        self.assertEqual(second.command_id, 2)

    def test_execution_status_echoes_command_segment_and_gripper_ack(self):
        trajectory = make_buffer()
        trajectory.submit(
            ARM_JOINT_NAMES,
            [[0.0] * 6, [0.1] * 6],
            [0.0, 1.0],
            [0.0] * 6,
            7.5,
            segment="lift",
        )
        gripper = GripperCommandBuffer(0.0, 0.07)
        gripper.submit(0.042, 7.75)
        status = format_execution_status(
            trajectory,
            gripper,
            physical_owner="trajectory_hold",
            measured_aperture=0.043,
        )
        self.assertTrue(status.startswith("active;owner=trajectory"))
        for expected in (
            "command_id=1",
            "segment=lift",
            "trajectory_received_at=7.500000",
            "gripper=accepted:0.0420",
            "gripper_command_id=1",
            "gripper_command_aperture=0.0420",
            "gripper_received_at=7.750000",
            "aperture=0.0430",
        ):
            self.assertIn(expected, status.split(";"))

    def test_bridge_reads_segment_from_joint_trajectory_header(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        self.assertIn("segment=_msg.header.frame_id", source)
        self.assertIn("format_execution_status(", source)


if __name__ == "__main__":
    unittest.main()
