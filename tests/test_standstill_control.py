import ast
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SIM_SCRIPTS = ROOT / "scripts/sim"
sys.path.insert(0, str(SIM_SCRIPTS))

from standstill_control import (  # noqa: E402
    ParkingBrakeConfig,
    StandstillCommandGate,
    StandstillGateConfig,
    WheelParkingBrake,
    select_joint_complement,
)


WAREHOUSE = SIM_SCRIPTS / "warehouse_nav.py"
BRINGUP = ROOT / "scripts/nav/bringup.sh"
RESTART = ROOT / "scripts/nav/restart_all.sh"
POLICY_DT_S = 0.02


def make_brake(transition_ticks=4):
    return WheelParkingBrake(ParkingBrakeConfig(
        drive_stiffness=0.0,
        drive_damping=5.0,
        park_stiffness=20.0,
        park_damping=9.0,
        transition_ticks=transition_ticks,
    ))


def park_gate(gate):
    decision = None
    while decision is None or not decision.engage:
        decision = gate.update(0.0, 0.0, 0.0, dt_s=POLICY_DT_S)
    assert decision is not None and decision.engage
    return decision


class StandstillCommandGateTest(unittest.TestCase):
    def test_defaults_use_separate_si_thresholds(self):
        config = StandstillGateConfig.from_environ({})

        self.assertEqual(config.linear_enter_mps, 0.005)
        self.assertEqual(config.linear_exit_mps, 0.015)
        self.assertEqual(config.angular_enter_rps, 0.005)
        self.assertEqual(config.angular_exit_rps, 0.015)
        self.assertEqual(config.enter_duration_s, 0.50)
        self.assertEqual(config.exit_duration_s, 0.04)

    def test_true_zero_requires_stable_window_before_parking(self):
        gate = StandstillCommandGate(StandstillGateConfig())

        for _ in range(24):
            decision = gate.update(0.0, 0.0, 0.0, dt_s=POLICY_DT_S)
            self.assertFalse(decision.parked)
            self.assertFalse(decision.engage)
        decision = gate.update(0.0, 0.0, 0.0, dt_s=POLICY_DT_S)

        self.assertTrue(decision.parked)
        self.assertTrue(decision.engage)
        self.assertEqual(decision.region, "enter")

    def test_manipulation_yaw_releases_parking_without_dimensional_norm(self):
        gate = StandstillCommandGate(StandstillGateConfig())
        park_gate(gate)

        first = gate.update(0.0, 0.0, 0.1087, dt_s=POLICY_DT_S)
        second = gate.update(0.0, 0.0, 0.1087, dt_s=POLICY_DT_S)

        self.assertTrue(first.parked)
        self.assertFalse(first.release)
        self.assertFalse(second.parked)
        self.assertTrue(second.release)
        self.assertEqual(second.linear_speed_mps, 0.0)
        self.assertAlmostEqual(second.angular_speed_rps, 0.1087)

    def test_low_linear_servo_and_normal_navigation_both_release(self):
        for command in ((0.026, 0.0, 0.0), (0.4, 0.0, 1.4)):
            gate = StandstillCommandGate(StandstillGateConfig())
            park_gate(gate)
            gate.update(*command, dt_s=POLICY_DT_S)
            decision = gate.update(*command, dt_s=POLICY_DT_S)
            self.assertTrue(decision.release)
            self.assertFalse(decision.parked)

    def test_signed_low_speed_controller_boundaries_never_repark(self):
        commands = (
            (0.0261, 0.0, 0.0),
            (-0.0261, 0.0, 0.0),
            (0.0, 0.0261, 0.0),
            (0.0, 0.0, 0.0301),
            (0.0, 0.0, -0.0524),
        )
        for command in commands:
            gate = StandstillCommandGate(StandstillGateConfig())
            park_gate(gate)
            released = False
            for _ in range(50):
                decision = gate.update(*command, dt_s=POLICY_DT_S)
                released = released or decision.release
                self.assertFalse(decision.engage)
            self.assertTrue(released)
            self.assertFalse(decision.parked)

    def test_reparks_only_after_a_new_stable_zero_window(self):
        gate = StandstillCommandGate(StandstillGateConfig())
        park_gate(gate)
        for _ in range(2):
            released = gate.update(0.4, 0.0, 0.0, dt_s=POLICY_DT_S)
        self.assertTrue(released.release)

        for _ in range(24):
            decision = gate.update(0.0, 0.0, 0.0, dt_s=POLICY_DT_S)
            self.assertFalse(decision.parked)
        decision = gate.update(0.0, 0.0, 0.0, dt_s=POLICY_DT_S)

        self.assertTrue(decision.engage)
        self.assertTrue(decision.parked)

    def test_hysteresis_band_cancels_partial_debounce_without_chatter(self):
        gate = StandstillCommandGate(StandstillGateConfig(
            enter_duration_s=0.06, exit_duration_s=0.04,
        ))
        gate.update(0.0, 0.0, 0.0, dt_s=POLICY_DT_S)
        gate.update(0.0, 0.0, 0.0, dt_s=POLICY_DT_S)

        band = gate.update(0.010, 0.0, 0.010, dt_s=POLICY_DT_S)

        self.assertEqual(band.region, "hysteresis")
        self.assertEqual(gate.enter_elapsed_s, 0.0)
        self.assertEqual(gate.exit_elapsed_s, 0.0)
        self.assertFalse(band.parked)

    def test_debounce_durations_are_independent_of_policy_cadence(self):
        for dt_s in (0.01, 0.02, 0.05):
            gate = StandstillCommandGate(StandstillGateConfig(
                enter_duration_s=0.50, exit_duration_s=0.10,
            ))
            elapsed = 0.0
            decision = None
            while decision is None or not decision.engage:
                decision = gate.update(0.0, 0.0, 0.0, dt_s=dt_s)
                elapsed += dt_s
            self.assertGreaterEqual(elapsed + 1e-12, 0.50)
            self.assertLess(elapsed, 0.50 + dt_s + 1e-12)

            elapsed = 0.0
            while not decision.release:
                decision = gate.update(0.2, 0.0, 0.0, dt_s=dt_s)
                elapsed += dt_s
            self.assertGreaterEqual(elapsed + 1e-12, 0.10)
            self.assertLess(elapsed, 0.10 + dt_s + 1e-12)

    def test_invalid_or_stale_source_can_force_immediate_parking(self):
        gate = StandstillCommandGate(StandstillGateConfig())

        forced = gate.force_park()
        repeated = gate.force_park()

        self.assertTrue(forced.engage)
        self.assertTrue(forced.parked)
        self.assertEqual(forced.region, "forced")
        self.assertFalse(repeated.engage)
        self.assertTrue(repeated.parked)

    def test_environment_overrides_and_invalid_configs_fail_closed(self):
        config = StandstillGateConfig.from_environ({
            "GO2W_STANDSTILL_LINEAR_ENTER_MPS": "0.001",
            "GO2W_STANDSTILL_LINEAR_EXIT_MPS": "0.010",
            "GO2W_STANDSTILL_ANGULAR_ENTER_RPS": "0.002",
            "GO2W_STANDSTILL_ANGULAR_EXIT_RPS": "0.020",
            "GO2W_STANDSTILL_ENTER_DURATION_S": "0.60",
            "GO2W_STANDSTILL_EXIT_DURATION_S": "0.06",
        })
        self.assertEqual(
            config,
            StandstillGateConfig(0.001, 0.010, 0.002, 0.020, 0.60, 0.06),
        )

        with self.assertRaisesRegex(ValueError, "linear.*below exit"):
            StandstillGateConfig(linear_enter_mps=0.02, linear_exit_mps=0.01)
        with self.assertRaisesRegex(ValueError, "angular.*below exit"):
            StandstillGateConfig(angular_enter_rps=0.02, angular_exit_rps=0.01)
        with self.assertRaisesRegex(ValueError, "exit_duration_s must be positive"):
            StandstillGateConfig(exit_duration_s=0.0)
        with self.assertRaisesRegex(ValueError, "finite cmd_vel"):
            StandstillCommandGate(StandstillGateConfig()).update(
                float("nan"), 0.0, 0.0, dt_s=POLICY_DT_S,
            )
        with self.assertRaisesRegex(ValueError, "dt_s must be positive"):
            StandstillCommandGate(StandstillGateConfig()).update(
                0.0, 0.0, 0.0, dt_s=0.0,
            )


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

    def test_runtime_wires_encoder_hold_and_complete_platform_state(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        helper_source = (SIM_SCRIPTS / "standstill_control.py").read_text(encoding="utf-8")
        ast.parse(source)

        for env_name in (
            "GO2W_STANDSTILL_WHEEL_KP",
            "GO2W_STANDSTILL_WHEEL_DAMPING",
            "GO2W_STANDSTILL_GAIN_TICKS",
            "GO2W_STANDSTILL_LINEAR_ENTER_MPS",
            "GO2W_STANDSTILL_LINEAR_EXIT_MPS",
            "GO2W_STANDSTILL_ANGULAR_ENTER_RPS",
            "GO2W_STANDSTILL_ANGULAR_EXIT_RPS",
            "GO2W_STANDSTILL_ENTER_DURATION_S",
            "GO2W_STANDSTILL_EXIT_DURATION_S",
        ):
            self.assertIn(env_name, helper_source)

        for contract in (
            "parking_brake.engage(",
            "standstill_gate.update(",
            "dt_s=2.0 * physics_dt",
            "standstill_gate.force_park()",
            "non-finite cmd_vel rejected; forcing parking",
            "def _apply_wheel_gains(",
            "wheel_actuator.stiffness.fill_(float(stiffness))",
            "wheel_actuator.damping.fill_(float(damping))",
            'policy_cache["wheel_position"]',
            '"/go2w/joint_states"',
            "select_joint_complement(",
            "robot.joint_names, piper_joint_ids or ()",
            "platform_js.position = robot.data.joint_pos",
        ):
            self.assertIn(contract, source)
        self.assertNotIn("cmd_norm = math.sqrt(vx * vx + vy * vy + wz * wz)", source)

        # Parking feedback must remain proprioceptive; GT stays an output oracle.
        engage_start = source.index("parking_brake.engage(")
        engage_end = source.index("# 柔性进入", engage_start)
        self.assertIn("robot.data.joint_pos", source[engage_start:engage_end])
        self.assertNotIn("root_pos", source[engage_start:engage_end])

    def test_runtime_entrypoints_forward_parking_configuration_and_reset_stops(self):
        source = WAREHOUSE.read_text(encoding="utf-8")
        entrypoint = BRINGUP.read_text(encoding="utf-8")
        for env_name in (
            "GO2W_POLICY_WHEEL_DAMPING",
            "GO2W_STANDSTILL_WHEEL_KP",
            "GO2W_STANDSTILL_WHEEL_DAMPING",
            "GO2W_STANDSTILL_GAIN_TICKS",
            "GO2W_STANDSTILL_LINEAR_ENTER_MPS",
            "GO2W_STANDSTILL_LINEAR_EXIT_MPS",
            "GO2W_STANDSTILL_ANGULAR_ENTER_RPS",
            "GO2W_STANDSTILL_ANGULAR_EXIT_RPS",
            "GO2W_STANDSTILL_ENTER_DURATION_S",
            "GO2W_STANDSTILL_EXIT_DURATION_S",
        ):
            self.assertIn(f'-e {env_name}=', entrypoint)
        gate_defaults = {
            "GO2W_STANDSTILL_LINEAR_ENTER_MPS": "0.005",
            "GO2W_STANDSTILL_LINEAR_EXIT_MPS": "0.015",
            "GO2W_STANDSTILL_ANGULAR_ENTER_RPS": "0.005",
            "GO2W_STANDSTILL_ANGULAR_EXIT_RPS": "0.015",
            "GO2W_STANDSTILL_ENTER_DURATION_S": "0.50",
            "GO2W_STANDSTILL_EXIT_DURATION_S": "0.04",
        }
        for env_name, default in gate_defaults.items():
            self.assertIn(
                f'-e {env_name}="${{{env_name}:-{default}}}"',
                entrypoint,
            )

        restart = RESTART.read_text(encoding="utf-8")
        self.assertIn('bash "$HERE/bringup.sh" teardown', restart)
        self.assertIn('exec bash "$HERE/bringup.sh" up', restart)

        reset_start = source.index('if reset_req["pending"]:')
        reset_end = source.index("# Process arm commands", reset_start)
        reset_block = source[reset_start:reset_end]
        self.assertIn("policy_cache.clear()", reset_block)
        self.assertIn(
            'cmd.update(vx=0.0, wz=0.0, t=float("-inf"), valid=False)',
            reset_block,
        )
        self.assertIn('vy_cmd["v"] = 0.0', reset_block)
        self.assertIn('policy_cache["legs"]', reset_block)
        self.assertIn('policy_cache["wheels"]', reset_block)


if __name__ == "__main__":
    unittest.main()
