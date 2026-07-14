"""Pure validation/arbitration contracts for the external PiPER joint-command face.

Z-Manip M3 route B ([RULING] CEO 2026-07-14): the z_manip grasp executor owns the
arm through a NEW ``/piper/joint_cmd`` (sensor_msgs/JointState) subscription in
``warehouse_nav.py``.  This module owns everything about that face that can be
stated without Isaac/torch/ROS — command validation, owner arbitration, and
per-tick rate limiting — mirroring the ``standstill_control.py`` precedent (pure
logic here, a thin runtime adapter in the main loop).

Owner priority (single exclusive writer per tick, CEO-approved):

    built-in grasp running  >  external fresh  >  named_pose

* "built-in grasp running": the legacy in-Isaac ``PiperGraspController`` debug
  face (``/piper/grasp_cmd``) — only alive when explicitly triggered; its
  semantics are untouched.
* "external fresh": a validated ``/piper/joint_cmd`` target was received within
  ``fresh_sim_s`` SIM seconds (default 0.5, the same freshness contract as the
  ``/manip/cmd_vel`` CMDMUX arbitration).  Publishers must therefore re-send at
  > 2 Hz sim to hold ownership; silence falls back to named_pose.
* "named_pose": the ``/piper/named_pose`` pose-table write (existing default).

Validation (reject loudly, never half-apply):
* joint NAMES are matched as a set against the runtime's authoritative order
  (``grasp.arm_names + grasp.grip_names``) and the positions are REORDERED to
  it — JointState is name-keyed by ROS convention, and ``find_joints`` order is
  not guaranteed to be numeric order (see warehouse_nav.py pose-tensor note);
* every position must be finite (NaN/inf rejected);
* every position must lie inside the articulation's joint position limits.

Rate limiting: an accepted external target is approached from the previously
written effective target at most ``dq_max`` per physics tick per joint
(``per_tick_dq_limits``) — a jump command produces bounded motion, exactly like
the built-in controller's ``DQ_MAX`` clamp (0.02 rad @ 50 Hz = 1 rad/s).
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Mapping, Optional, Sequence

# ------------------------------------------------------------------ owner tags
OWNER_GRASP = "grasp"            # built-in PiperGraspController (debug face)
OWNER_EXTERNAL = "external"      # /piper/joint_cmd (z_manip grasp executor)
OWNER_NAMED_POSE = "named_pose"  # /piper/named_pose pose table (default)

# ------------------------------------------------------------------- env knobs
FRESH_ENV = "GO2W_ARM_EXT_FRESH_S"          # external freshness window (sim s)
ARM_VEL_ENV = "GO2W_ARM_EXT_VEL_MAX"        # arm joint rate cap (rad/s)
GRIP_VEL_ENV = "GO2W_ARM_EXT_GRIP_VEL_MAX"  # gripper prismatic rate cap (m/s)

# Defaults: freshness mirrors the /manip/cmd_vel CMDMUX window (0.5 sim-s);
# the arm rate cap mirrors the built-in controller's effective 1 rad/s
# (DQ_MAX 0.02 @ 50 Hz); the gripper cap closes the full 0.07 m stroke in
# ~0.7 s — gentle, and slower than any grasp-phase timeout.
DEFAULT_FRESH_SIM_S = 0.5
DEFAULT_ARM_VEL_RAD_S = 1.0
DEFAULT_GRIP_VEL_M_S = 0.1

# Limit-check slack: a caller echoing back our own reported target must never
# be rejected for a float round-trip wobble.
_LIMIT_EPS = 1e-6


def _finite_positive(value: float, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f"{name} must be finite and positive, got {value!r}")
    return parsed


@dataclass(frozen=True)
class ArmGateParams:
    """Validated knobs for the external joint-command gate."""

    fresh_sim_s: float
    arm_vel_rad_s: float
    grip_vel_m_s: float

    def __post_init__(self) -> None:
        for field_name in ("fresh_sim_s", "arm_vel_rad_s", "grip_vel_m_s"):
            object.__setattr__(
                self, field_name,
                _finite_positive(getattr(self, field_name), field_name),
            )


def gate_params_from_environ(environ: Mapping[str, str]) -> ArmGateParams:
    """Build validated gate params from the environment (bringup 1c precedent)."""
    return ArmGateParams(
        fresh_sim_s=float(environ.get(FRESH_ENV, str(DEFAULT_FRESH_SIM_S))),
        arm_vel_rad_s=float(environ.get(ARM_VEL_ENV, str(DEFAULT_ARM_VEL_RAD_S))),
        grip_vel_m_s=float(environ.get(GRIP_VEL_ENV, str(DEFAULT_GRIP_VEL_M_S))),
    )


# ---------------------------------------------------------------- validation
def validate_joint_command(
    names: Sequence[str],
    positions: Sequence[float],
    expected_names: Sequence[str],
    limits_lo: Sequence[float],
    limits_hi: Sequence[float],
) -> tuple[Optional[list[float]], str]:
    """Validate one JointState command against the runtime joint contract.

    Args:
        names: joint names as received on the wire.
        positions: positions aligned with ``names``.
        expected_names: the runtime's authoritative joint order
            (``grasp.arm_names + grasp.grip_names``); the output is reordered
            to THIS order regardless of wire order (JointState is name-keyed).
        limits_lo / limits_hi: per-joint position limits in ``expected_names``
            order (from ``robot.data.joint_pos_limits``).

    Returns:
        ``(ordered_positions, "")`` on success — a plain float list in
        ``expected_names`` order — or ``(None, reason)`` on rejection.
        A rejected command must be dropped whole; never applied partially.
    """
    expected = list(expected_names)
    if len(names) != len(positions):
        return None, (f"name/position length mismatch "
                      f"({len(names)} names vs {len(positions)} positions)")
    if len(names) != len(expected):
        return None, (f"expected {len(expected)} joints "
                      f"{expected}, got {len(names)}")
    if len(set(names)) != len(names):
        return None, f"duplicate joint names in command: {sorted(names)}"
    by_name = {}
    for n, p in zip(names, positions):
        if n not in expected:
            return None, f"unknown joint {n!r}; expected exactly {expected}"
        by_name[n] = float(p)
    # set equality follows from: same length, no dups, no unknowns.
    ordered: list[float] = []
    for i, n in enumerate(expected):
        v = by_name[n]
        if not math.isfinite(v):
            return None, f"non-finite position for {n!r}: {v!r}"
        lo, hi = float(limits_lo[i]), float(limits_hi[i])
        if v < lo - _LIMIT_EPS or v > hi + _LIMIT_EPS:
            return None, (f"{n!r} position {v:.4f} outside limits "
                          f"[{lo:.4f}, {hi:.4f}]")
        ordered.append(v)
    return ordered, ""


# --------------------------------------------------------------- arbitration
def resolve_owner(
    grasp_running: bool,
    external_stamp_sim: Optional[float],
    sim_now: float,
    fresh_sim_s: float = DEFAULT_FRESH_SIM_S,
) -> str:
    """Single-owner arbitration for the arm's 8-joint target this tick.

    Args:
        grasp_running: built-in controller status starts with ``running:``.
        external_stamp_sim: sim time the last VALIDATED external command was
            received (None = never).
        sim_now: current sim time.
        fresh_sim_s: freshness window (sim seconds).

    Returns:
        One of :data:`OWNER_GRASP` / :data:`OWNER_EXTERNAL` /
        :data:`OWNER_NAMED_POSE`.  Priority: grasp > external fresh >
        named_pose.  An external stamp from the "future" (clock reset /
        sim restart) is treated as stale — never trust a negative age.
    """
    if grasp_running:
        return OWNER_GRASP
    if external_stamp_sim is not None:
        age = sim_now - external_stamp_sim
        if 0.0 <= age < fresh_sim_s:
            return OWNER_EXTERNAL
    return OWNER_NAMED_POSE


# --------------------------------------------------------------- rate limiting
def per_tick_dq_limits(
    physics_dt: float,
    n_arm: int,
    n_grip: int,
    arm_vel_rad_s: float = DEFAULT_ARM_VEL_RAD_S,
    grip_vel_m_s: float = DEFAULT_GRIP_VEL_M_S,
) -> list[float]:
    """Per-joint max target step per physics tick (rad or m, by joint kind)."""
    dt = _finite_positive(physics_dt, "physics_dt")
    if n_arm < 0 or n_grip < 0:
        raise ValueError(f"joint counts must be >= 0, got {n_arm}/{n_grip}")
    return ([_finite_positive(arm_vel_rad_s, "arm_vel_rad_s") * dt] * n_arm
            + [_finite_positive(grip_vel_m_s, "grip_vel_m_s") * dt] * n_grip)


def rate_limit_step(
    prev: Sequence[float],
    target: Sequence[float],
    dq_max: Sequence[float],
) -> list[float]:
    """One bounded step from ``prev`` toward ``target`` (per-joint clamp).

    Returns ``prev + clip(target - prev, -dq_max, +dq_max)`` element-wise.
    All three sequences must have equal length (raises ValueError otherwise —
    a length mismatch is a wiring bug, never something to guess around).
    """
    if not (len(prev) == len(target) == len(dq_max)):
        raise ValueError(
            f"length mismatch: prev={len(prev)} target={len(target)} "
            f"dq_max={len(dq_max)}")
    out: list[float] = []
    for p, t, m in zip(prev, target, dq_max):
        step = max(-float(m), min(float(m), float(t) - float(p)))
        out.append(float(p) + step)
    return out


if __name__ == "__main__":  # pragma: no cover — tiny self-check, no Isaac needed
    params = gate_params_from_environ(os.environ)
    print(f"[arm_command_gate] params={params}")
    demo, why = validate_joint_command(
        ["a", "b"], [0.0, 0.5], ["a", "b"], [-1.0, 0.0], [1.0, 1.0])
    print(f"[arm_command_gate] demo validate -> {demo} ({why or 'ok'})")
