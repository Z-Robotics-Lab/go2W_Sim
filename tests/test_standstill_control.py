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
RESTART = ROOT / "scripts/nav/restart_all.sh"


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
        ):
            self.assertIn(env_name, helper_source)

        for contract in (
            "parking_brake.engage(",
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
        ):
            self.assertIn(f'-e {env_name}=', entrypoint)

        restart = RESTART.read_text(encoding="utf-8")
        self.assertIn('bash "$HERE/bringup.sh" teardown', restart)
        self.assertIn('exec bash "$HERE/bringup.sh" up', restart)

        reset_start = source.index('if reset_req["pending"]:')
        reset_end = source.index("# Process arm commands", reset_start)
        reset_block = source[reset_start:reset_end]
        self.assertIn("policy_cache.clear()", reset_block)
        self.assertIn('cmd.update(vx=0.0, wz=0.0, t=float("-inf"))', reset_block)
        self.assertIn('vy_cmd["v"] = 0.0', reset_block)
        self.assertIn('policy_cache["legs"]', reset_block)
        self.assertIn('policy_cache["wheels"]', reset_block)


if __name__ == "__main__":
    unittest.main()
