import json
import math
import re
import sys
import unittest
from pathlib import Path

import numpy as np


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
    EndpointConvergenceConfig,
    format_execution_status,
    GripperCommandBuffer,
    GripperValidationError,
    JointTrajectoryBuffer,
    TrajectoryValidationError,
)
from wrist_camera import (  # noqa: E402
    camera_update_elapsed_dt,
    require_new_camera_frame,
)


WAREHOUSE = SIM_SCRIPTS / "warehouse_nav.py"
CAMERA = SIM_SCRIPTS / "wrist_camera.py"
BRINGUP = ROOT / "scripts/nav/bringup.sh"
SCENE_CONFIG = ROOT / "configs/manip_office_scene.json"


def make_buffer(
    endpoint_convergence: EndpointConvergenceConfig | None = None,
) -> JointTrajectoryBuffer:
    return JointTrajectoryBuffer(
        ARM_JOINT_NAMES,
        {name: (-3.0, 3.0) for name in ARM_JOINT_NAMES},
        {name: 2.0 for name in ARM_JOINT_NAMES},
        endpoint_convergence=endpoint_convergence,
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
            "math.isclose(camera_update_dt, CAM_PUBLISH_PERIOD,", source)
        self.assertIn(
            "d435.update(camera_update_dt, force_recompute=True)", source)
        self.assertNotIn("d435.update(physics_dt)", source)

    def test_camera_internal_gate_cannot_drop_long_running_strided_updates(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        self.assertIn("CAM_SENSOR_UPDATE_PERIOD = 0.0", source)
        self.assertIn("update_period=CAM_SENSOR_UPDATE_PERIOD", source)
        self.assertNotIn("update_period=CAM_PUBLISH_PERIOD", source)

        def missed_updates(update_period: float, cycles: int = 3000) -> list[int]:
            # Exact model of SensorBase's float32 timestamp gate.  The legacy
            # 0.1 s gate first fails at cycle 319 despite 0.1 s update calls.
            timestamp = np.float32(0.0)
            timestamp_last_update = np.float32(0.0)
            misses = []
            for cycle in range(cycles):
                timestamp = np.float32(timestamp + np.float32(0.1))
                elapsed = np.float32(
                    timestamp - timestamp_last_update + np.float32(1.0e-6))
                if elapsed >= np.float32(update_period):
                    timestamp_last_update = timestamp
                else:
                    misses.append(cycle)
            return misses

        legacy_misses = missed_updates(0.1)
        self.assertEqual(legacy_misses[0], 319)
        self.assertGreater(len(legacy_misses), 400)
        self.assertEqual(missed_updates(0.0), [])

    def test_camera_update_elapsed_dt_rejects_invalid_clock_inputs(self):
        for physics_dt, stride in ((0.0, 10), (float("nan"), 10), (0.01, 0),
                                   (0.01, True)):
            with self.subTest(physics_dt=physics_dt, stride=stride):
                with self.assertRaises(ValueError):
                    camera_update_elapsed_dt(physics_dt, stride)

    def test_camera_publication_requires_a_strictly_new_render_frame(self):
        self.assertEqual(require_new_camera_frame(None, 1), 1)
        self.assertEqual(require_new_camera_frame(1, 2), 2)
        for previous, current, error in (
            (2, 2, RuntimeError),
            (2, 1, RuntimeError),
            (None, 0, ValueError),
            (None, True, ValueError),
        ):
            with self.subTest(previous=previous, current=current):
                with self.assertRaises(error):
                    require_new_camera_frame(previous, current)

        source = WAREHOUSE.read_text(encoding="utf-8")
        self.assertIn("camera_data = d435.data", source)
        self.assertIn("wc.require_new_camera_frame(", source)
        self.assertIn("except RuntimeError as error:", source)
        self.assertIn("and camera_frame_ready", source)
        self.assertNotIn('"rgb" in d435.data.output', source)

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
        halfway = buffer.sample(
            12.5, [0.0] * 6, [0.0] * 6, feedback_at=12.49)
        for actual, expected in zip(halfway.positions, final):
            self.assertAlmostEqual(actual, expected * 0.5)
        self.assertFalse(halfway.done)
        settling = buffer.sample(
            13.0, final, [0.0] * 6, feedback_at=13.0)
        self.assertFalse(settling.done)
        self.assertEqual(buffer.status, "active")
        self.assertEqual(buffer.phase, "settling")
        done = buffer.sample(
            13.2, final, [0.0] * 6, feedback_at=13.2)
        self.assertTrue(done.done)
        self.assertEqual(buffer.status, "succeeded")

    def test_elapsed_duration_cannot_succeed_without_measured_endpoint_convergence(self):
        buffer = make_buffer(EndpointConvergenceConfig(
            position_tolerance_rad=0.03,
            velocity_tolerance_rad_s=0.08,
            dwell_s=0.10,
            timeout_s=0.50,
            feedback_max_age_s=0.10,
        ))
        endpoint = [0.2] * 6
        buffer.submit(
            ARM_JOINT_NAMES,
            [[0.0] * 6, endpoint],
            [0.0, 1.0],
            [0.0] * 6,
            4.0,
            segment="approach",
        )

        at_duration = buffer.sample(
            5.0, [0.0] * 6, [0.0] * 6, feedback_at=5.0)
        self.assertFalse(at_duration.done)
        self.assertEqual(buffer.status, "active")
        self.assertAlmostEqual(buffer.endpoint_position_error_rad, 0.2)

        timed_out = buffer.sample(
            5.5, [0.1] * 6, [0.0] * 6, feedback_at=5.5)
        self.assertTrue(timed_out.done)
        self.assertEqual(buffer.status, "rejected:endpoint_not_converged")
        self.assertEqual(timed_out.positions, (0.1,) * 6)

    def test_endpoint_gate_requires_velocity_and_continuous_feedback_dwell(self):
        buffer = make_buffer(EndpointConvergenceConfig(
            position_tolerance_rad=0.03,
            velocity_tolerance_rad_s=0.08,
            dwell_s=0.20,
            timeout_s=1.0,
            feedback_max_age_s=0.15,
        ))
        endpoint = [0.1] * 6
        buffer.submit(
            ARM_JOINT_NAMES,
            [[0.0] * 6, endpoint],
            [0.0, 1.0],
            [0.0] * 6,
            0.0,
            segment="lift",
        )

        moving = buffer.sample(
            1.0, endpoint, [0.2] * 6, feedback_at=1.0)
        self.assertFalse(moving.done)
        first_quiet = buffer.sample(
            1.05, endpoint, [0.0] * 6, feedback_at=1.05)
        self.assertFalse(first_quiet.done)
        self.assertAlmostEqual(buffer.settle_dwell_s, 0.0)
        buffer.sample(1.14, endpoint, [0.0] * 6, feedback_at=1.14)
        almost = buffer.sample(
            1.24, endpoint, [0.0] * 6, feedback_at=1.24)
        self.assertFalse(almost.done)
        self.assertAlmostEqual(buffer.settle_dwell_s, 0.19)

        # One excursion resets the continuous interval; prior quiet samples
        # cannot be accumulated across it.
        excursion = buffer.sample(
            1.25, [0.14] * 6, [0.0] * 6, feedback_at=1.25)
        self.assertFalse(excursion.done)
        self.assertAlmostEqual(buffer.settle_dwell_s, 0.0)
        buffer.sample(1.30, endpoint, [0.0] * 6, feedback_at=1.30)
        buffer.sample(1.40, endpoint, [0.0] * 6, feedback_at=1.40)
        done = buffer.sample(1.50, endpoint, [0.0] * 6, feedback_at=1.50)
        self.assertTrue(done.done)
        self.assertEqual(buffer.status, "succeeded")

    def test_reused_or_stale_feedback_cannot_satisfy_dwell(self):
        buffer = make_buffer(EndpointConvergenceConfig(
            position_tolerance_rad=0.03,
            velocity_tolerance_rad_s=0.08,
            dwell_s=0.10,
            timeout_s=1.0,
            feedback_max_age_s=0.20,
        ))
        endpoint = [0.1] * 6
        buffer.submit(
            ARM_JOINT_NAMES,
            [[0.0] * 6, endpoint],
            [0.0, 1.0],
            [0.0] * 6,
            0.0,
            segment="transit",
        )
        buffer.sample(1.0, endpoint, [0.0] * 6, feedback_at=1.0)
        reused = buffer.sample(1.19, endpoint, [0.0] * 6, feedback_at=1.0)
        self.assertFalse(reused.done)
        self.assertAlmostEqual(buffer.settle_dwell_s, 0.0)

        stale = buffer.sample(1.21, endpoint, [0.0] * 6, feedback_at=1.0)
        self.assertTrue(stale.done)
        self.assertEqual(buffer.status, "rejected:feedback_stale")

    def test_feedback_time_regression_fails_closed_at_measured_hold(self):
        buffer = make_buffer()
        buffer.submit(
            ARM_JOINT_NAMES,
            [[0.0] * 6, [0.1] * 6],
            [0.0, 1.0],
            [0.0] * 6,
            2.0,
            segment="carry",
        )
        buffer.sample(2.2, [0.02] * 6, [0.0] * 6, feedback_at=2.19)
        failed = buffer.sample(
            2.3, [0.03] * 6, [0.0] * 6, feedback_at=2.18)
        self.assertTrue(failed.done)
        self.assertEqual(buffer.status, "rejected:feedback_time_regressed")
        self.assertEqual(failed.positions, (0.02,) * 6)

    def test_endpoint_convergence_config_has_validated_environment_overrides(self):
        config = EndpointConvergenceConfig.from_environ({
            "GO2W_PIPER_ENDPOINT_POSITION_TOLERANCE_RAD": "0.025",
            "GO2W_PIPER_ENDPOINT_VELOCITY_TOLERANCE_RAD_S": "0.07",
            "GO2W_PIPER_ENDPOINT_DWELL_S": "0.3",
            "GO2W_PIPER_ENDPOINT_TIMEOUT_S": "1.5",
            "GO2W_PIPER_FEEDBACK_MAX_AGE_S": "0.12",
        })
        self.assertEqual(config.position_tolerance_rad, 0.025)
        self.assertEqual(config.velocity_tolerance_rad_s, 0.07)
        self.assertEqual(config.dwell_s, 0.3)
        self.assertEqual(config.timeout_s, 1.5)
        self.assertEqual(config.feedback_max_age_s, 0.12)
        with self.assertRaisesRegex(ValueError, "dwell_s must not exceed"):
            EndpointConvergenceConfig(dwell_s=2.0, timeout_s=1.0)
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            EndpointConvergenceConfig(feedback_max_age_s=float("nan"))

        bringup = BRINGUP.read_text(encoding="utf-8")
        for name in (
            "GO2W_PIPER_ENDPOINT_POSITION_TOLERANCE_RAD",
            "GO2W_PIPER_ENDPOINT_VELOCITY_TOLERANCE_RAD_S",
            "GO2W_PIPER_ENDPOINT_DWELL_S",
            "GO2W_PIPER_ENDPOINT_TIMEOUT_S",
            "GO2W_PIPER_FEEDBACK_MAX_AGE_S",
        ):
            self.assertIn(f'-e {name}="${{{name}:-', bringup)

    def test_bridge_proves_completion_from_timestamped_joint_feedback(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        self.assertIn("measured_arm_velocity = robot.data.joint_vel", source)
        self.assertIn("feedback_at=max(0.0, sim_t[\"now\"] - physics_dt)", source)
        self.assertNotIn(
            'print("[PIPER_EXEC] trajectory succeeded -> final hold"', source)

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
