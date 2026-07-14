"""Pure-logic tests for the external PiPER joint-command gate (Z-Manip M3 route B).

``ArmCommandGateTest`` exercises the pure module (validation / owner
arbitration / rate limiting / env params) with zero Isaac/torch/ROS imports —
the ``standstill_control`` precedent.

``RuntimeWiringContractTest`` asserts (source-level, like the parking-brake
suite) that ``warehouse_nav.py`` actually wires the gate: the
``/piper/joint_cmd`` subscription exists, the owner block routes through
``resolve_owner``, the external branch rate-limits through
``rate_limit_step``, and the ``/piper/cmd`` report still publishes the
post-arbitration effective target (G-c contract unchanged).
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIM_SCRIPTS = ROOT / "scripts/sim"
sys.path.insert(0, str(SIM_SCRIPTS))

import arm_command_gate as acg  # noqa: E402


EXPECTED = ["piper_joint1", "piper_joint2", "piper_joint3",
            "piper_joint4", "piper_joint5", "piper_joint6",
            "piper_joint7", "piper_joint8"]
LO = [-2.618, 0.0, -2.967, -1.745, -1.22, -2.0944, 0.0, -0.035]
HI = [2.618, 3.14, 0.0, 1.745, 1.22, 2.0944, 0.035, 0.0]
GOOD = [0.0, 0.8, -1.2, 0.0, 0.0, 0.0, 0.02, -0.02]


class ValidationTest(unittest.TestCase):
    def test_good_command_passes_and_keeps_order(self):
        out, why = acg.validate_joint_command(EXPECTED, GOOD, EXPECTED, LO, HI)
        self.assertEqual(why, "")
        self.assertEqual(out, GOOD)

    def test_wire_order_is_reordered_by_name(self):
        perm = list(reversed(range(8)))
        names = [EXPECTED[i] for i in perm]
        pos = [GOOD[i] for i in perm]
        out, why = acg.validate_joint_command(names, pos, EXPECTED, LO, HI)
        self.assertEqual(why, "")
        self.assertEqual(out, GOOD)  # reordered back to expected order

    def test_missing_joint_rejected(self):
        out, why = acg.validate_joint_command(
            EXPECTED[:7], GOOD[:7], EXPECTED, LO, HI)
        self.assertIsNone(out)
        self.assertIn("expected 8", why)

    def test_unknown_joint_rejected(self):
        names = EXPECTED[:7] + ["piper_joint9"]
        out, why = acg.validate_joint_command(names, GOOD, EXPECTED, LO, HI)
        self.assertIsNone(out)
        self.assertIn("unknown joint", why)

    def test_duplicate_joint_rejected(self):
        names = EXPECTED[:7] + ["piper_joint1"]
        out, why = acg.validate_joint_command(names, GOOD, EXPECTED, LO, HI)
        self.assertIsNone(out)
        self.assertIn("duplicate", why)

    def test_length_mismatch_rejected(self):
        out, why = acg.validate_joint_command(EXPECTED, GOOD[:7], EXPECTED, LO, HI)
        self.assertIsNone(out)
        self.assertIn("length mismatch", why)

    def test_nan_and_inf_rejected(self):
        for bad in (math.nan, math.inf, -math.inf):
            pos = list(GOOD)
            pos[3] = bad
            out, why = acg.validate_joint_command(EXPECTED, pos, EXPECTED, LO, HI)
            self.assertIsNone(out, f"{bad} must be rejected")
            self.assertIn("non-finite", why)

    def test_out_of_limits_rejected_both_sides(self):
        for idx, val in ((1, 3.20), (2, 0.10), (6, 0.05), (7, 0.01)):
            pos = list(GOOD)
            pos[idx] = val
            out, why = acg.validate_joint_command(EXPECTED, pos, EXPECTED, LO, HI)
            self.assertIsNone(out, f"joint {idx} at {val} must be rejected")
            self.assertIn("outside limits", why)

    def test_exact_limit_boundary_passes(self):
        pos = list(LO)  # every joint parked AT its lower limit
        out, why = acg.validate_joint_command(EXPECTED, pos, EXPECTED, LO, HI)
        self.assertEqual(why, "")
        self.assertEqual(out, [float(v) for v in LO])


class OwnerArbitrationTest(unittest.TestCase):
    """Priority matrix: grasp running > external fresh > named_pose."""

    def test_grasp_beats_fresh_external(self):
        self.assertEqual(
            acg.resolve_owner(True, external_stamp_sim=9.9, sim_now=10.0),
            acg.OWNER_GRASP)

    def test_grasp_beats_named_pose(self):
        self.assertEqual(acg.resolve_owner(True, None, 10.0), acg.OWNER_GRASP)

    def test_fresh_external_beats_named_pose(self):
        self.assertEqual(
            acg.resolve_owner(False, external_stamp_sim=9.8, sim_now=10.0),
            acg.OWNER_EXTERNAL)

    def test_stale_external_falls_back_to_named_pose(self):
        self.assertEqual(
            acg.resolve_owner(False, external_stamp_sim=9.4, sim_now=10.0),
            acg.OWNER_NAMED_POSE)

    def test_freshness_window_boundary_is_stale(self):
        # age == window must NOT hold ownership (strict <).
        self.assertEqual(
            acg.resolve_owner(False, external_stamp_sim=9.5, sim_now=10.0,
                              fresh_sim_s=0.5),
            acg.OWNER_NAMED_POSE)

    def test_never_received_external_is_named_pose(self):
        self.assertEqual(acg.resolve_owner(False, None, 10.0),
                         acg.OWNER_NAMED_POSE)

    def test_future_stamp_is_stale_not_owner(self):
        # clock reset / sim restart: a stamp ahead of now is never fresh.
        self.assertEqual(
            acg.resolve_owner(False, external_stamp_sim=11.0, sim_now=10.0),
            acg.OWNER_NAMED_POSE)


class RateLimitTest(unittest.TestCase):
    def test_big_jump_is_clamped_per_joint(self):
        prev = [0.0, 0.0]
        tgt = [1.0, -1.0]
        out = acg.rate_limit_step(prev, tgt, [0.01, 0.02])
        self.assertEqual(out, [0.01, -0.02])

    def test_small_step_passes_through(self):
        out = acg.rate_limit_step([0.5], [0.505], [0.01])
        self.assertAlmostEqual(out[0], 0.505)

    def test_converges_to_target(self):
        q = [0.0]
        for _ in range(200):
            q = acg.rate_limit_step(q, [1.0], [0.01])
        self.assertAlmostEqual(q[0], 1.0, places=6)

    def test_length_mismatch_raises(self):
        with self.assertRaises(ValueError):
            acg.rate_limit_step([0.0], [1.0, 2.0], [0.01, 0.01])

    def test_per_tick_dq_limits_split_arm_grip(self):
        dq = acg.per_tick_dq_limits(0.01, 6, 2, arm_vel_rad_s=1.0,
                                    grip_vel_m_s=0.1)
        self.assertEqual(len(dq), 8)
        for v in dq[:6]:
            self.assertAlmostEqual(v, 0.01)
        for v in dq[6:]:
            self.assertAlmostEqual(v, 0.001)

    def test_per_tick_dq_limits_rejects_bad_dt(self):
        for bad in (0.0, -1.0, math.nan):
            with self.assertRaises(ValueError):
                acg.per_tick_dq_limits(bad, 6, 2)


class GateParamsTest(unittest.TestCase):
    def test_defaults(self):
        p = acg.gate_params_from_environ({})
        self.assertEqual(p.fresh_sim_s, acg.DEFAULT_FRESH_SIM_S)
        self.assertEqual(p.arm_vel_rad_s, acg.DEFAULT_ARM_VEL_RAD_S)
        self.assertEqual(p.grip_vel_m_s, acg.DEFAULT_GRIP_VEL_M_S)

    def test_env_overrides(self):
        p = acg.gate_params_from_environ({
            acg.FRESH_ENV: "0.8", acg.ARM_VEL_ENV: "0.5",
            acg.GRIP_VEL_ENV: "0.05"})
        self.assertEqual((p.fresh_sim_s, p.arm_vel_rad_s, p.grip_vel_m_s),
                         (0.8, 0.5, 0.05))

    def test_invalid_env_rejected_loudly(self):
        for env in ({acg.FRESH_ENV: "0"}, {acg.ARM_VEL_ENV: "-1"},
                    {acg.GRIP_VEL_ENV: "nan"}):
            with self.assertRaises(ValueError):
                acg.gate_params_from_environ(env)
        with self.assertRaises(ValueError):
            acg.gate_params_from_environ({acg.FRESH_ENV: "abc"})


class RuntimeWiringContractTest(unittest.TestCase):
    """Source-contract assertions on warehouse_nav.py (parking-brake precedent).

    The main loop cannot be imported host-side (Isaac/torch), so — exactly like
    ``test_standstill_control.py`` — we assert the wiring exists in source: the
    subscription, the arbitration call, the rate-limit call in the external
    branch, and the unchanged effective-target report.
    """

    @classmethod
    def setUpClass(cls):
        cls.src = (SIM_SCRIPTS / "warehouse_nav.py").read_text(encoding="utf-8")

    def test_joint_cmd_subscription_exists(self):
        self.assertIn('"/piper/joint_cmd"', self.src)
        self.assertIn("on_joint_cmd", self.src)

    def test_validation_is_the_gate_module(self):
        self.assertIn("arm_command_gate", self.src)
        self.assertIn("validate_joint_command", self.src)

    def test_owner_block_routes_through_resolve_owner(self):
        self.assertIn("resolve_owner", self.src)
        self.assertIn("OWNER_GRASP", self.src)
        self.assertIn("OWNER_EXTERNAL", self.src)

    def test_external_branch_is_rate_limited(self):
        self.assertIn("rate_limit_step", self.src)
        self.assertIn("per_tick_dq_limits", self.src)

    def test_effective_target_report_unchanged(self):
        # G-c contract: /piper/cmd reports the post-arbitration target.
        self.assertIn("jc.position = eff_arm_tgt.tolist()", self.src)

    def test_rejected_commands_are_logged_not_silent(self):
        self.assertIn("joint_cmd rejected", self.src)


if __name__ == "__main__":
    unittest.main()
