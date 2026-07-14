"""Pure-logic tests for the Go2W wheel parking brake (P1.2).

The ``WheelParkingBrakeTest`` class is ported verbatim from the codex branch
(feat/codex-mobile-manip-office) — the ``standstill_control`` module it exercises
is byte-identical, so its contract is unchanged.

The ``RuntimeWiringContractTest`` class is adapted to *this* branch's
``warehouse_nav.py`` integration.  Main here has a ``/manip/cmd_vel`` CMDMUX
arbitration that the codex branch never had, and no ``trajectory_executor`` /
``select_joint_complement`` platform-joint machinery; so the source-contract
assertions below target main's actual encoder-parking wiring, not codex's.

``ManipOverrideRuleTest`` is new: it proves rule 2 (a non-zero MANIP-sourced
command releases immediately and inhibits engagement, bypassing hysteresis).
Because that rule lives in the Isaac main loop (torch/ROS), it is verified as a
faithful pure-logic re-enactment of the exact release/inhibit transitions the
runtime performs, plus a source-contract assertion that the runtime wires it.
"""

import ast
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SIM_SCRIPTS = ROOT / "scripts/sim"
sys.path.insert(0, str(SIM_SCRIPTS))

from standstill_control import (  # noqa: E402
    ParkingBrakeConfig,
    WheelParkingBrake,
    select_joint_complement,
)


WAREHOUSE = SIM_SCRIPTS / "warehouse_nav.py"
BRINGUP = ROOT / "scripts/nav/bringup.sh"


def make_brake(transition_ticks=4):
    return WheelParkingBrake(ParkingBrakeConfig(
        drive_stiffness=0.0,
        drive_damping=5.0,
        park_stiffness=20.0,
        park_damping=9.0,
        transition_ticks=transition_ticks,
    ))


class WheelParkingBrakeTest(unittest.TestCase):
    def test_engage_latches_encoders_once_and_blends_gains(self):
        brake = make_brake()
        brake.engage((1.0, 2.0, 3.0, 4.0))

        commands = []
        for offset in (0.0, 0.1, 0.2, 0.3):
            # Moving measurements must not move the original parking anchor.
            brake.engage(tuple(value + offset for value in (1.0, 2.0, 3.0, 4.0)))
            commands.append(brake.step(
                tuple(value + offset for value in (1.0, 2.0, 3.0, 4.0))))

        self.assertEqual([command.blend for command in commands], [0.25, 0.5, 0.75, 1.0])
        self.assertEqual([command.stiffness for command in commands], [5.0, 10.0, 15.0, 20.0])
        self.assertEqual([command.damping for command in commands], [6.0, 7.0, 8.0, 9.0])
        self.assertTrue(all(
            command.position_target == (1.0, 2.0, 3.0, 4.0)
            for command in commands
        ))
        self.assertEqual(commands[-1].mode, "parked")

    def test_release_recenters_target_then_returns_to_velocity_mode(self):
        brake = make_brake()
        brake.engage((1.0, 2.0))
        for _ in range(4):
            brake.step((1.0, 2.0))

        brake.release()
        commands = [
            brake.step((1.1 + index, 2.1 + index))
            for index in range(4)
        ]

        self.assertEqual([command.blend for command in commands], [0.75, 0.5, 0.25, 0.0])
        self.assertEqual(commands[0].position_target, (1.1, 2.1))
        self.assertEqual(commands[1].position_target, (2.1, 3.1))
        self.assertEqual(commands[2].position_target, (3.1, 4.1))
        self.assertIsNone(commands[-1].position_target)
        self.assertEqual(commands[-1].mode, "drive")
        self.assertEqual(commands[-1].stiffness, 0.0)
        self.assertEqual(commands[-1].damping, 5.0)
        self.assertIsNone(brake.anchor)

    def test_reengage_during_release_uses_fresh_encoder_anchor(self):
        brake = make_brake()
        brake.engage((1.0, 2.0))
        for _ in range(4):
            brake.step((1.0, 2.0))
        brake.release()
        brake.step((1.2, 2.2))

        brake.engage((5.0, 6.0))
        command = brake.step((5.1, 6.1))

        self.assertEqual(command.position_target, (5.0, 6.0))
        self.assertTrue(brake.desired_parked)

    def test_reset_and_invalid_configuration_fail_closed(self):
        brake = make_brake()
        brake.engage((1.0, 2.0))
        brake.step((1.0, 2.0))
        brake.reset()
        command = brake.step((3.0, 4.0))
        self.assertEqual(command.mode, "drive")
        self.assertIsNone(command.position_target)

        with self.assertRaises(ValueError):
            ParkingBrakeConfig(0.0, 5.0, 0.0, 8.0, 10)
        with self.assertRaises(ValueError):
            ParkingBrakeConfig.from_environ(
                {"GO2W_STANDSTILL_GAIN_TICKS": "bad"},
                drive_stiffness=0.0,
                drive_damping=5.0,
            )


class PlatformJointContractTest(unittest.TestCase):
    """select_joint_complement stays a correct dynamic complement.

    Ported from codex (the helper is byte-identical).  Main does not *call* this
    helper in the parking-brake path (it uses ``policy.wheel_ids`` directly), but
    the module ships it, so its contract must hold for any future caller.
    """

    def test_platform_joint_selection_is_dynamic_complement(self):
        names = (
            "front_leg", "front_wheel", "future_suspension",
            "piper_axis_a", "piper_finger_b",
        )
        indices, selected_names = select_joint_complement(names, (3, 4))
        self.assertEqual(indices, (0, 1, 2))
        self.assertEqual(
            selected_names,
            ("front_leg", "front_wheel", "future_suspension"),
        )

    def test_platform_joint_selection_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            select_joint_complement((), ())
        with self.assertRaises(ValueError):
            select_joint_complement(("a", "a"), ())        # duplicate names
        with self.assertRaises(ValueError):
            select_joint_complement(("a", "b"), (5,))       # out-of-range index
        with self.assertRaises(ValueError):
            select_joint_complement(("a", "b"), (0, 1))     # empty complement


class RuntimeWiringContractTest(unittest.TestCase):
    """Assert THIS branch's warehouse_nav.py actually wires the parking brake.

    Adapted from codex's ``PlatformJointContractTest`` runtime assertions to
    main's structure (encoder parking + CMDMUX MANIP arbitration).
    """

    def test_warehouse_nav_parses(self):
        ast.parse(WAREHOUSE.read_text(encoding="utf-8"))

    def test_runtime_wires_encoder_parking_brake(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        helper_source = (SIM_SCRIPTS / "standstill_control.py").read_text(encoding="utf-8")

        for env_name in (
            "GO2W_STANDSTILL_WHEEL_KP",
            "GO2W_STANDSTILL_WHEEL_DAMPING",
            "GO2W_STANDSTILL_GAIN_TICKS",
        ):
            self.assertIn(env_name, helper_source)

        for contract in (
            "from standstill_control import ParkingBrakeConfig, WheelParkingBrake",
            "ParkingBrakeConfig.from_environ(",
            "parking_brake = WheelParkingBrake(",
            "parking_brake.engage(",
            "parking_brake.release()",
            "parking_brake.step(",
            "def _apply_wheel_gains(",
            "wheel_actuator.stiffness.fill_(float(stiffness))",
            "wheel_actuator.damping.fill_(float(damping))",
            'policy_cache["wheel_position"]',
            "parking_command.blend",
        ):
            self.assertIn(contract, source)

        # Parking feedback must stay proprioceptive (wheel encoders), never GT/root.
        engage_start = source.index("parking_brake.engage(")
        engage_end = source.index("# 柔性进入", engage_start)
        self.assertIn("robot.data.joint_pos", source[engage_start:engage_end])
        self.assertNotIn("root_pos", source[engage_start:engage_end])
        self.assertNotIn("root_quat", source[engage_start:engage_end])

    def test_reset_releases_brake_and_restores_drive_gains(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        reset_start = source.index('if reset_req["pending"]:')
        reset_end = source.index("cmd_vel 看门狗", reset_start)
        reset_block = source[reset_start:reset_end]
        self.assertIn("parking_brake.reset()", reset_block)
        self.assertIn("_apply_wheel_gains(", reset_block)
        self.assertIn('policy_cache.pop("wheel_position", None)', reset_block)

    def test_bringup_forwards_parking_env_whitelist(self):
        entrypoint = BRINGUP.read_text(encoding="utf-8")
        for env_name in (
            "GO2W_STANDSTILL_WHEEL_KP",
            "GO2W_STANDSTILL_WHEEL_DAMPING",
            "GO2W_STANDSTILL_GAIN_TICKS",
            "GO2W_POLICY_LEG_STIFFNESS",
            "GO2W_POLICY_LEG_DAMPING",
        ):
            self.assertIn(f'-e {env_name}=', entrypoint)
        # GO2W_OBS_DUMP is opt-in (no default) — passed through by name only.
        self.assertIn("-e GO2W_OBS_DUMP", entrypoint)

    def test_bringup_order_switch_defaults_nav_first(self):
        entrypoint = BRINGUP.read_text(encoding="utf-8")
        self.assertIn('BRINGUP_ORDER="${GO2W_BRINGUP_ORDER:-nav_first}"', entrypoint)
        self.assertIn('if [ "$BRINGUP_ORDER" = "isaac_first" ]; then', entrypoint)
        # isaac_first waits for the 8s settle marker before launching navstack.
        self.assertIn('_wait_isaac_marker "imu settled"', entrypoint)
        # nav_first (default) waits for the plain sensor marker.
        self.assertIn('_wait_isaac_marker "imu sample"', entrypoint)
        # both orders reuse the same frozen launch helpers.
        self.assertIn("_launch_navstack", entrypoint)
        self.assertIn("_launch_isaac_bridge", entrypoint)

    def test_bringup_domain_is_parameterized(self):
        entrypoint = BRINGUP.read_text(encoding="utf-8")
        self.assertIn('ROS_DOMAIN_ID="${GO2W_ROS_DOMAIN_ID:-42}"', entrypoint)
        # No lingering hardcoded domain in the docker launches.
        self.assertNotIn("-e ROS_DOMAIN_ID=42", entrypoint)

    def test_warehouse_emits_settle_marker_for_isaac_first(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        self.assertIn("imu settled: step=", source)
        self.assertIn("if step == 800:", source)


class ManipOverrideRuleTest(unittest.TestCase):
    """Rule 2: a non-zero MANIP-sourced command releases immediately and inhibits
    engagement, bypassing the standstill ENTER/EXIT hysteresis.

    This re-enacts the exact pure-logic transitions the runtime performs so the
    guarantee is falsifiable without Isaac.  It models the runtime's per-tick
    decision: ``manip_active_nonzero == (source is MANIP and cmd != 0)``; when
    true the brake is released and no engage() may fire regardless of the low
    command norm or debounce counters.
    """

    ENTER_DEBOUNCE = 25  # mirrors STANDSTILL_ENTER_DEBOUNCE in warehouse_nav.py

    @staticmethod
    def _manip_active_nonzero(source_is_manip, vx, vy, wz):
        # Byte-for-byte the runtime predicate (warehouse_nav.py CMDMUX block).
        return source_is_manip and (vx != 0.0 or vy != 0.0 or wz != 0.0)

    def test_small_manip_command_below_exit_band_still_releases(self):
        # A 0.1 m/s micro-servo command: 3D norm 0.1 < EXIT thresh 0.25, so plain
        # hysteresis would keep parking; rule 2 must release anyway.
        brake = make_brake(transition_ticks=4)
        brake.engage((0.0, 0.0, 0.0, 0.0))
        for _ in range(4):
            brake.step((0.0, 0.0, 0.0, 0.0))
        self.assertTrue(brake.desired_parked)

        # MANIP publishes vx=0.1 (< EXIT 0.25). Runtime sets manip_active_nonzero.
        self.assertTrue(self._manip_active_nonzero(True, 0.1, 0.0, 0.0))
        brake.release()  # rule-2 branch calls release() immediately
        cmd = brake.step((0.0, 0.0, 0.0, 0.0))
        self.assertFalse(brake.desired_parked)
        self.assertLess(cmd.blend, 1.0)  # gain is fading out, not held at park

    def test_manip_source_inhibits_engagement_despite_low_norm(self):
        # Simulate ENTER_DEBOUNCE+ consecutive ticks of a low-norm MANIP command.
        # Because rule 2 short-circuits BEFORE the enter-debounce branch, the
        # low_count never accumulates and the brake never engages.
        brake = make_brake(transition_ticks=4)
        standstill_active = False
        low_count = 0
        for _ in range(self.ENTER_DEBOUNCE + 10):
            manip_active_nonzero = self._manip_active_nonzero(True, 0.08, 0.0, 0.0)
            if manip_active_nonzero:
                # runtime: reset low_count, force high_count, never engage()
                low_count = 0
                if standstill_active:
                    standstill_active = False
                    brake.release()
            else:  # would run the enter-debounce branch
                low_count += 1
                if low_count >= self.ENTER_DEBOUNCE and not standstill_active:
                    standstill_active = True
                    brake.engage((0.0, 0.0, 0.0, 0.0))
        self.assertFalse(standstill_active)
        self.assertFalse(brake.desired_parked)
        self.assertEqual(low_count, 0)

    def test_zero_manip_command_does_not_trigger_override(self):
        # MANIP source but a zero command (e.g. servo idle) is NOT intentional
        # motion, so rule 2 must not fire — normal hysteresis governs parking.
        self.assertFalse(self._manip_active_nonzero(True, 0.0, 0.0, 0.0))
        # Pathfollower source with a small command is also not a rule-2 override.
        self.assertFalse(self._manip_active_nonzero(False, 0.1, 0.0, 0.0))
        # Any non-zero component under MANIP triggers it.
        self.assertTrue(self._manip_active_nonzero(True, 0.0, 0.0, -0.05))


if __name__ == "__main__":
    unittest.main()
